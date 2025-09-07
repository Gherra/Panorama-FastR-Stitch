classdef ImageBlender < handle
    % ImageBlender - Image warping and blending for seamless panoramas
    %
    % This class handles the final stage of panorama creation:
    % warping images according to computed homographies and blending
    % them together to minimize visible seams. I implement linear
    % feathering for smooth transitions between overlapping regions.
    
    properties (Access = private)
        config  % Configuration parameters
    end
    
    methods
        function obj = ImageBlender(config)
            obj.config = config;
        end
        
        function panorama = blendPair(obj, img1, img2, H)
            % Blend two images using the homography H that maps img2 to img1's frame
            % The first image stays fixed, the second is warped to align
            
            % Convert homography to MATLAB's projective2d format
            % Note: MATLAB uses transposed convention
            tform = projective2d(H');
            
            [h1, w1, ~] = size(img1);
            [h2, w2, ~] = size(img2);
            
            % Calculate output canvas bounds
            % Need to find where the corners of both images will land
            [xlim1, ylim1] = outputLimits(projective2d(eye(3)), [1 w1], [1 h1]);
            [xlim2, ylim2] = outputLimits(tform, [1 w2], [1 h2]);
            
            % The panorama needs to contain both images
            xMin = min([xlim1(1), xlim2(1), 1]);
            xMax = max([xlim1(2), xlim2(2), w1]);
            yMin = min([ylim1(1), ylim2(1), 1]);
            yMax = max([ylim1(2), ylim2(2), h1]);
            
            width = round(xMax - xMin);
            height = round(yMax - yMin);
            
            % Define the output coordinate system
            outputView = imref2d([height, width], [xMin xMax], [yMin yMax]);
            
            % Warp both images to the output canvas
            % img1 gets identity transform (stays in place)
            warpedImg1 = imwarp(img1, projective2d(eye(3)), 'OutputView', outputView);
            warpedImg2 = imwarp(img2, tform, 'OutputView', outputView);
            
            % Create weight masks for blending
            % These track which pixels come from which image
            mask1 = imwarp(ones(h1, w1), projective2d(eye(3)), 'OutputView', outputView);
            mask2 = imwarp(ones(h2, w2), tform, 'OutputView', outputView);
            
            % Apply the selected blending method
            switch lower(obj.config.blendMethod)
                case 'none'
                    % Simple overlay - no smooth blending
                    panorama = obj.blendNone(warpedImg1, warpedImg2, mask1, mask2);
                case 'linear'
                    % Linear feathering - my default choice
                    panorama = obj.blendLinear(warpedImg1, warpedImg2, mask1, mask2);
                case 'multiband'
                    % Multiband blending using Laplacian pyramids
                    panorama = obj.blendMultiband(warpedImg1, warpedImg2, mask1, mask2);
                otherwise
                    panorama = obj.blendLinear(warpedImg1, warpedImg2, mask1, mask2);
            end
        end
        
        function panorama = blendMultiple(obj, images, transforms)
            % Blend multiple images with their cumulative transforms
            % Each transform maps that image to the first image's coordinate frame
            
            numImages = length(images);
            
            % Convert transform matrices to projective2d objects
            tforms = repmat(projective2d(), numImages, 1);
            for i = 1:numImages
                tforms(i) = projective2d(transforms{i}');
            end
            
            % Find the bounding box that contains all warped images
            xMin = inf; xMax = -inf;
            yMin = inf; yMax = -inf;
            
            for i = 1:numImages
                [h, w, ~] = size(images{i});
                [xlim, ylim] = outputLimits(tforms(i), [1 w], [1 h]);
                xMin = min(xMin, xlim(1));
                xMax = max(xMax, xlim(2));
                yMin = min(yMin, ylim(1));
                yMax = max(yMax, ylim(2));
            end
            
            % Add padding to ensure no clipping
            xMin = xMin - 10;
            xMax = xMax + 10;
            yMin = yMin - 10;
            yMax = yMax + 10;
            
            width = round(xMax - xMin);
            height = round(yMax - yMin);
            
            outputView = imref2d([height, width], [xMin xMax], [yMin yMax]);
            
            % Initialize accumulators for weighted averaging
            accumImage = zeros(height, width, 3);
            accumMask = zeros(height, width);
            
            % Warp and accumulate each image with distance-based weights
            for i = 1:numImages
                % Warp this image to the output canvas
                warpedImg = imwarp(images{i}, tforms(i), 'OutputView', outputView);
                
                % Create mask for valid pixels
                [h, w, ~] = size(images{i});
                mask = imwarp(ones(h, w), tforms(i), 'OutputView', outputView);
                
                % Apply distance weighting for smooth blending
                % Pixels near image edges get lower weight
                if strcmpi(obj.config.blendMethod, 'linear')
                    mask = obj.applyDistanceWeight(mask);
                end
                
                % Accumulate weighted contributions
                accumImage = accumImage + warpedImg .* mask;
                accumMask = accumMask + mask;
            end
            
            % Normalize by total weight to get final colors
            accumMask(accumMask == 0) = 1;  % Prevent division by zero
            panorama = accumImage ./ accumMask;
            
            % Remove black borders
            panorama = obj.cropToValid(panorama);
        end
        
        function panorama = blendNone(obj, img1, img2, mask1, mask2)
            % No blending - second image simply overwrites first
            % This shows the raw alignment quality
            
            panorama = img1;
            overlapMask = mask2 > 0;
            
            % Copy img2 pixels where it's valid
            for c = 1:3
                channel = panorama(:,:,c);
                channel(overlapMask) = img2(overlapMask + (c-1)*numel(overlapMask));
                panorama(:,:,c) = channel;
            end
        end
        
        function panorama = blendLinear(obj, img1, img2, mask1, mask2)
            % Linear blending with distance-based feathering
            % This is my preferred method - gives smooth transitions
            
            % Apply distance transform to create smooth weight falloff
            weight1 = obj.applyDistanceWeight(mask1);
            weight2 = obj.applyDistanceWeight(mask2);
            
            % Normalize weights so they sum to 1
            totalWeight = weight1 + weight2;
            totalWeight(totalWeight == 0) = 1;
            
            % Weighted average of the two images
            panorama = (img1 .* weight1 + img2 .* weight2) ./ totalWeight;
        end
        
        function panorama = blendMultiband(obj, img1, img2, mask1, mask2)
            % Multiband blending using Laplacian pyramids
            % This preserves both low and high frequency details
            % Based on Burt & Adelson's classic paper
            
            numLevels = 4;  % Number of pyramid levels
            
            % Decompose images into frequency bands
            pyr1 = obj.buildLaplacianPyramid(img1, numLevels);
            pyr2 = obj.buildLaplacianPyramid(img2, numLevels);
            
            % Build Gaussian pyramids for the masks
            maskPyr1 = obj.buildGaussianPyramid(mask1, numLevels);
            maskPyr2 = obj.buildGaussianPyramid(mask2, numLevels);
            
            % Blend each frequency band separately
            blendedPyr = cell(numLevels, 1);
            for level = 1:numLevels
                % Normalize masks at this resolution
                totalMask = maskPyr1{level} + maskPyr2{level};
                totalMask(totalMask == 0) = 1;
                
                % Blend this frequency band
                blendedPyr{level} = (pyr1{level} .* maskPyr1{level} + ...
                                    pyr2{level} .* maskPyr2{level}) ./ totalMask;
            end
            
            % Reconstruct full image from blended pyramid
            panorama = obj.reconstructFromPyramid(blendedPyr);
            
            % Ensure valid pixel range
            panorama = max(0, min(1, panorama));
        end
        
        function weight = applyDistanceWeight(obj, mask)
            % Create smooth weight that falls off at image boundaries
            % This prevents hard edges in the blend
            
            if all(mask(:) == 0) || all(mask(:) == 1)
                weight = mask;
                return;
            end
            
            % Compute distance from edge using distance transform
            % Each pixel gets the distance to nearest zero pixel
            dist = bwdist(~(mask > 0.5));
            
            % Normalize distances to [0, 1]
            maxDist = max(dist(:));
            if maxDist > 0
                weight = dist / maxDist;
            else
                weight = mask;
            end
            
            % Apply non-linear falloff for smoother blend
            % Square root gives nice gradual transition
            weight = weight .^ 0.5;
        end
        
        function pyr = buildLaplacianPyramid(obj, img, numLevels)
            % Build Laplacian pyramid (band-pass decomposition)
            % Each level contains details at a specific frequency
            
            pyr = cell(numLevels, 1);
            current = img;
            
            for level = 1:numLevels-1
                % Low-pass filter and downsample
                down = imresize(current, 0.5, 'bilinear');
                
                % Upsample back to original size
                up = imresize(down, size(current, 1:2), 'bilinear');
                
                % Laplacian = difference between levels (band-pass)
                pyr{level} = current - up;
                
                current = down;
            end
            
            % Last level contains low-frequency residual
            pyr{numLevels} = current;
        end
        
        function pyr = buildGaussianPyramid(obj, img, numLevels)
            % Build Gaussian pyramid (low-pass decomposition)
            % Used for mask pyramids in multiband blending
            
            pyr = cell(numLevels, 1);
            pyr{1} = img;
            
            for level = 2:numLevels
                % Smooth and downsample
                filtered = imgaussfilt(pyr{level-1}, 1);
                pyr{level} = imresize(filtered, 0.5, 'bilinear');
            end
        end
        
        function img = reconstructFromPyramid(obj, pyr)
            % Reconstruct image from Laplacian pyramid
            % Start from coarsest level and add details
            
            numLevels = length(pyr);
            img = pyr{numLevels};  % Start with low-frequency base
            
            for level = numLevels-1:-1:1
                % Upsample and add high-frequency details
                upsampled = imresize(img, size(pyr{level}, 1:2), 'bilinear');
                img = upsampled + pyr{level};
            end
        end
        
        function cropped = cropToValid(obj, img)
            % Remove black borders from panorama
            % Makes the final result look cleaner
            
            gray = rgb2gray(img);
            
            % Find pixels that contain actual image data
            validMask = gray > 0.01;
            
            % Find bounding box of valid region
            [rows, cols] = find(validMask);
            
            if isempty(rows) || isempty(cols)
                cropped = img;
                return;
            end
            
            % Add small margin
            minRow = max(1, min(rows) - 5);
            maxRow = min(size(img, 1), max(rows) + 5);
            minCol = max(1, min(cols) - 5);
            maxCol = min(size(img, 2), max(cols) + 5);
            
            cropped = img(minRow:maxRow, minCol:maxCol, :);
        end
    end
end