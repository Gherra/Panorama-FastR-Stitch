classdef FastDetector < handle
    % FastDetector - Custom implementation of FAST corner detection
    % 
    % This class implements the FAST (Features from Accelerated Segment Test)
    % algorithm for corner detection. I've added Harris corner response filtering
    % to create a more robust variant called FASTR.
    %
    % The FAST algorithm works by examining a circle of 16 pixels around
    % each candidate point. If a contiguous arc of pixels are all significantly
    % brighter or darker than the center, it's classified as a corner.
    
    properties (Access = private)
        config           % Configuration parameters
        circleOffsets    % Pre-computed pixel offsets for the Bresenham circle
    end
    
    methods
        function obj = FastDetector(config)
            % Initialize the detector with configuration
            obj.config = config;
            
            % Pre-compute the 16 pixel positions on a circle of radius 3
            % These offsets follow the Bresenham circle algorithm
            % Starting from top (12 o'clock) and going clockwise
            obj.circleOffsets = [
                0, -3;  1, -3;  2, -2;  3, -1;   % Top-right quadrant
                3,  0;  3,  1;  2,  2;  1,  3;   % Bottom-right quadrant
                0,  3; -1,  3; -2,  2; -3,  1;   % Bottom-left quadrant
               -3,  0; -3, -1; -2, -2; -1, -3    % Top-left quadrant
            ];
        end
        
        function points = detectFAST(obj, img)
            % Standard FAST detection without additional filtering
            % This gives us all corners that pass the FAST criterion
            points = obj.detectCore(img, obj.config.fastThreshold, obj.config.fastN);
        end
        
        function points = detectFASTR(obj, img)
            % FASTR - my enhanced version with Harris corner response filtering
            % This removes ambiguous corners that FAST might detect on edges
            
            % First, get all FAST corners
            points = obj.detectCore(img, obj.config.fastThreshold, obj.config.fastN);
            
            if isempty(points)
                return;
            end
            
            % Now I compute the Harris corner response at each detected point
            % This helps distinguish true corners from edge points
            R = obj.computeHarrisResponse(img);
            
            % Extract Harris values at the detected corner locations
            indices = sub2ind(size(img), points(:,2), points(:,1));
            harrisValues = R(indices);
            
            % Keep only corners with strong Harris response
            % This filters out weak or ambiguous corners
            strongCorners = harrisValues > obj.config.harrisThreshold;
            points = points(strongCorners, :);
            
            if obj.config.verbose
                % Report how many corners were filtered out
                totalDetected = sum(~strongCorners) + sum(strongCorners);
                fprintf('    FAST detected %d corners, kept %d after Harris filtering\n', ...
                        totalDetected, sum(strongCorners));
            end
        end
        
        function points = detectCore(obj, img, threshold, N)
            % Core FAST detection algorithm implementation
            % This is where the actual corner detection happens
            
            [height, width] = size(img);
            
            % Can't detect corners too close to image edges (need the circle)
            borderSize = 3;
            validRows = (borderSize + 1):(height - borderSize);
            validCols = (borderSize + 1):(width - borderSize);
            
            % Create a grid of all pixels we'll test for corner-ness
            [X, Y] = meshgrid(validCols, validRows);
            candidates = [X(:), Y(:)];
            numCandidates = size(candidates, 1);
            
            % Pre-allocate array for circle pixel intensities
            % Each row will store the 16 pixel values around one candidate
            circleIntensities = zeros(numCandidates, 16);
            
            % Extract intensities for all candidates at once (vectorized for speed)
            for i = 1:16
                dx = obj.circleOffsets(i, 1);
                dy = obj.circleOffsets(i, 2);
                
                % Calculate positions of circle pixels
                neighborX = candidates(:, 1) + dx;
                neighborY = candidates(:, 2) + dy;
                
                % Get intensities using linear indexing
                idx = sub2ind([height, width], neighborY, neighborX);
                circleIntensities(:, i) = img(idx);
            end
            
            % Get the center pixel intensities
            centerIdx = sub2ind([height, width], candidates(:, 2), candidates(:, 1));
            centerIntensities = img(centerIdx);
            
            % FAST criterion: pixels must be threshold brighter or darker
            upperThreshold = centerIntensities + threshold;
            lowerThreshold = centerIntensities - threshold;
            
            % Mark which circle pixels are significantly brighter/darker
            brighter = circleIntensities > upperThreshold;
            darker = circleIntensities < lowerThreshold;
            
            % Check for N contiguous bright or dark pixels
            % This is the key FAST criterion - we need an arc of similar pixels
            isCorner = obj.checkContiguous(brighter, darker, N);
            
            % Extract the corner coordinates
            points = candidates(isCorner, :);
            
            % If we detected too many corners, apply non-maximum suppression
            % This keeps only the strongest corners in each local region
            if size(points, 1) > 500
                points = obj.nonMaxSuppression(points, img, threshold);
            end
        end
        
        function isCorner = checkContiguous(obj, brighter, darker, N)
            % Check if there are N contiguous pixels that are all brighter OR darker
            % This implements the core FAST criterion
            
            numCandidates = size(brighter, 1);
            isCorner = false(numCandidates, 1);
            
            % To handle the circular nature, wrap the arrays
            % This lets us check arcs that cross the starting point
            brighterWrap = [brighter, brighter(:, 1:N-1)];
            darkerWrap = [darker, darker(:, 1:N-1)];
            
            % Check all possible starting positions for an N-pixel arc
            for startIdx = 1:16
                % Extract N consecutive pixels starting at this position
                brightSeq = brighterWrap(:, startIdx:startIdx+N-1);
                darkSeq = darkerWrap(:, startIdx:startIdx+N-1);
                
                % A corner has ALL N pixels brighter OR ALL N pixels darker
                allBright = all(brightSeq, 2);
                allDark = all(darkSeq, 2);
                
                % Mark as corner if either criterion is met
                isCorner = isCorner | allBright | allDark;
            end
        end
        
        function R = computeHarrisResponse(obj, img)
            % Compute Harris corner response for the entire image
            % This measures the "cornerness" at each pixel based on local gradients
            
            % First, compute image gradients using Sobel operators
            % These tell us the rate of intensity change in x and y directions
            [Ix, Iy] = imgradientxy(img, 'sobel');
            
            % Build the structure tensor (second moment matrix) components
            % These capture the local gradient distribution
            Ix2 = Ix.^2;      % Gradient energy in x direction
            Iy2 = Iy.^2;      % Gradient energy in y direction  
            Ixy = Ix .* Iy;   % Gradient correlation
            
            % Apply Gaussian weighting to aggregate local information
            % The window size is derived from the Harris threshold parameter
            windowSize = obj.config.harrisThreshold * 1000;
            windowSize = max(3, min(7, round(windowSize)));  % Keep between 3 and 7
            
            sigma = windowSize / 3;  % Standard deviation for Gaussian
            g = fspecial('gaussian', windowSize, sigma);
            
            % Smooth the structure tensor components
            Sx2 = imfilter(Ix2, g, 'replicate');
            Sy2 = imfilter(Iy2, g, 'replicate');
            Sxy = imfilter(Ixy, g, 'replicate');
            
            % Harris corner response: R = det(M) - k * trace(M)^2
            % Where M is the structure tensor [Sx2 Sxy; Sxy Sy2]
            % Large R indicates a corner, negative R indicates an edge
            k = 0.04;  % Harris sensitivity parameter (empirically determined)
            detM = Sx2 .* Sy2 - Sxy.^2;
            traceM = Sx2 + Sy2;
            R = detM - k * traceM.^2;
            
            % Normalize response to [0, 1] range for consistent thresholding
            R = (R - min(R(:))) / (max(R(:)) - min(R(:)) + eps);
        end
        
        function points = nonMaxSuppression(obj, points, img, threshold)
            % Non-maximum suppression to reduce corner density
            % Keeps only the locally strongest corners
            
            % Compute a strength measure for each detected corner
            strengths = zeros(size(points, 1), 1);
            for i = 1:size(points, 1)
                x = points(i, 1);
                y = points(i, 2);
                
                % Use local variance as corner strength
                % Stronger corners have more variation in their neighborhood
                localPatch = img(max(1, y-1):min(size(img,1), y+1), ...
                                 max(1, x-1):min(size(img,2), x+1));
                strengths(i) = var(localPatch(:));
            end
            
            % Sort corners by strength and keep only the top 500
            [~, idx] = sort(strengths, 'descend');
            maxPoints = 500;
            keepIdx = idx(1:min(maxPoints, length(idx)));
            points = points(keepIdx, :);
        end
    end
end