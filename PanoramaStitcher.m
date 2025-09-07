classdef PanoramaStitcher < handle
    % PanoramaStitcher - Main orchestrator for the image stitching pipeline
    % 
    % This class coordinates the entire panorama creation process:
    % 1. Feature detection using custom FAST implementation
    % 2. Feature description using SURF
    % 3. Robust matching with RANSAC
    % 4. Image warping and blending
    
    properties (Access = public)
        config          % Configuration parameters
        fastDetector    % Custom FAST detector instance
        matcher         % Feature matching and RANSAC module
        blender         % Image warping and blending module
    end
    
    methods
        function obj = PanoramaStitcher(config)
            % Initialize all pipeline components
            obj.config = config;
            
            % Create instances of each processing module
            obj.fastDetector = FastDetector(config);
            obj.matcher = FeatureMatcher(config);
            obj.blender = ImageBlender(config);
        end
        
        function [panorama, stats] = stitchImages(obj, imagePaths)
            % Main entry point for stitching multiple images
            % Handles both 2-image and multi-image panoramas
            
            % Load and preprocess the images (resize, format conversion)
            images = loadAndPrepImages(imagePaths);
            numImages = length(images);
            
            % Initialize statistics tracking
            stats = struct();
            stats.numImages = numImages;
            stats.avgFeatures = 0;
            stats.avgMatches = 0;
            stats.avgInlierRatio = 0;
            
            if numImages < 2
                error('Need at least 2 images to create a panorama');
            elseif numImages == 2
                % Simple case: just two images to stitch
                [panorama, pairStats] = obj.stitchPair(images{1}, images{2});
                stats.avgFeatures = pairStats.avgFeatures;
                stats.avgMatches = pairStats.numMatches;
                stats.avgInlierRatio = pairStats.inlierRatio;
            else
                % Complex case: chain multiple homographies
                [panorama, multiStats] = obj.stitchMultiple(images);
                stats = multiStats;
            end
            
            % Generate visualization showing statistics
            if obj.config.saveVisualizations
                obj.saveVisualization(panorama, stats);
            end
        end
        
        function [panorama, stats] = stitchPair(obj, img1, img2)
            % Stitch exactly two images together
            % This is the fundamental operation that gets chained for multiple images
            
            % Convert to grayscale since FAST works on intensity
            gray1 = rgb2gray(img1);
            gray2 = rgb2gray(img2);
            
            if obj.config.verbose
                fprintf('  Detecting corner features using FAST algorithm\n');
            end
            
            % Detect and describe features in both images
            [points1, features1] = obj.detectAndDescribe(gray1);
            [points2, features2] = obj.detectAndDescribe(gray2);
            
            % Track average number of features for statistics
            stats.avgFeatures = (size(points1, 1) + size(points2, 1)) / 2;
            
            if obj.config.verbose
                fprintf('  Found %d and %d features respectively\n', ...
                        size(points1, 1), size(points2, 1));
                fprintf('  Matching features using SURF descriptors\n');
            end
            
            % Find correspondences between the two sets of features
            [matches, matchStats] = obj.matcher.matchFeatures(...
                features1, features2, points1, points2);
            
            stats.numMatches = matchStats.numMatches;
            stats.inlierRatio = matchStats.inlierRatio;
            
            % Need at least 4 point correspondences to compute homography
            if matchStats.numInliers < 4
                error('Only found %d inliers, need at least 4 for homography', ...
                      matchStats.numInliers);
            end
            
            if obj.config.verbose
                fprintf('  Computing homography from %d inliers\n', matchStats.numInliers);
                fprintf('  Warping and blending images\n');
            end
            
            % Apply homography and blend the warped images
            panorama = obj.blender.blendPair(img1, img2, matchStats.H);
        end
        
        function [panorama, stats] = stitchMultiple(obj, images)
            % Stitch more than two images by chaining homographies
            % Images are processed sequentially: 1->2, 2->3, 3->4, etc.
            
            numImages = length(images);
            
            % First image is the reference (identity transform)
            transforms = cell(numImages, 1);
            transforms{1} = eye(3);
            
            % Accumulate statistics across all pairs
            totalFeatures = 0;
            totalMatches = 0;
            totalInlierRatio = 0;
            
            % Process each adjacent pair to build transformation chain
            for i = 2:numImages
                if obj.config.verbose
                    fprintf('  Processing image pair %d-%d\n', i-1, i);
                end
                
                gray1 = rgb2gray(images{i-1});
                gray2 = rgb2gray(images{i});
                
                % Detect features in both images
                [points1, features1] = obj.detectAndDescribe(gray1);
                [points2, features2] = obj.detectAndDescribe(gray2);
                
                totalFeatures = totalFeatures + size(points1, 1) + size(points2, 1);
                
                % Match features and estimate homography
                [~, matchStats] = obj.matcher.matchFeatures(...
                    features1, features2, points1, points2);
                
                totalMatches = totalMatches + matchStats.numMatches;
                totalInlierRatio = totalInlierRatio + matchStats.inlierRatio;
                
                % Chain transformations: each image transforms relative to first
                % H_1to3 = H_1to2 * H_2to3
                transforms{i} = transforms{i-1} * matchStats.H;
            end
            
            % Calculate average statistics
            stats.numImages = numImages;
            stats.avgFeatures = totalFeatures / (2 * (numImages - 1));
            stats.avgMatches = totalMatches / (numImages - 1);
            stats.avgInlierRatio = totalInlierRatio / (numImages - 1);
            
            if obj.config.verbose
                fprintf('  Blending all %d images into final panorama\n', numImages);
            end
            
            % Warp all images to common coordinate frame and blend
            panorama = obj.blender.blendMultiple(images, transforms);
        end
        
        function [points, features] = detectAndDescribe(obj, grayImg)
            % Detect keypoints and compute their descriptors
            % Combines FAST detection with SURF description
            
            % Use either FAST or FASTR (with Harris filtering) based on config
            if strcmpi(obj.config.detector, 'FASTR')
                points = obj.fastDetector.detectFASTR(grayImg);
            else
                points = obj.fastDetector.detectFAST(grayImg);
            end
            
            % No features detected - return empty
            if isempty(points)
                features = [];
                return;
            end
            
            % Convert to cornerPoints format for MATLAB's feature extraction
            cornerPts = cornerPoints(points);
            
            % Extract SURF descriptors at detected corners
            % SURF provides rotation and scale invariance
            [features, validPts] = extractFeatures(grayImg, cornerPts, ...
                'Method', obj.config.descriptor);
            
            % Some points might be too close to image boundary for descriptor
            points = validPts.Location;
        end
        
        function saveVisualization(obj, panorama, stats)
            % Create a summary visualization with the panorama and statistics
            
            fig = figure('Visible', 'off', 'Position', [100, 100, 1200, 600]);
            
            % Display the panorama
            subplot(1, 2, 1);
            imshow(panorama);
            title('Stitched Panorama');
            
            % Display processing statistics
            subplot(1, 2, 2);
            axis off;
            
            % Format statistics as text
            textY = 0.9;
            text(0.1, textY, 'Processing Statistics', 'FontSize', 14, 'FontWeight', 'bold');
            textY = textY - 0.15;
            
            text(0.1, textY, sprintf('Images stitched: %d', stats.numImages), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Avg features detected: %.0f', stats.avgFeatures), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Avg matches found: %.0f', stats.avgMatches), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Avg inlier ratio: %.1f%%', stats.avgInlierRatio * 100), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Detector: %s', obj.config.detector), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Descriptor: %s', obj.config.descriptor), 'FontSize', 12);
            textY = textY - 0.1;
            
            text(0.1, textY, sprintf('Blend: %s', obj.config.blendMethod), 'FontSize', 12);
            
            % Save the visualization
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            vizPath = fullfile(obj.config.outputDir, sprintf('viz_%s.png', timestamp));
            saveas(fig, vizPath);
            close(fig);
        end
    end
end