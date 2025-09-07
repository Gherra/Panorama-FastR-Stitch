classdef FeatureMatcher < handle
    % FeatureMatcher - Robust feature matching and homography estimation
    %
    % This class handles the critical task of finding correct correspondences
    % between features in two images and computing the geometric transformation
    % (homography) that aligns them. I use RANSAC to handle outliers.
    
    properties (Access = private)
        config  % Configuration parameters
    end
    
    methods
        function obj = FeatureMatcher(config)
            obj.config = config;
        end
        
        function [matches, stats] = matchFeatures(obj, features1, features2, points1, points2)
            % Match features between two images and compute the homography
            % Returns both the matched points and statistics about the matching quality
            
            stats = struct();
            
            % Handle edge case where no features were detected
            if isempty(features1) || isempty(features2)
                matches = [];
                stats.numMatches = 0;
                stats.numInliers = 0;
                stats.inlierRatio = 0;
                stats.H = eye(3);  % Identity transform as fallback
                return;
            end
            
            % Find putative matches using SURF descriptors
            % I use ratio test (Lowe's criterion) to filter ambiguous matches
            try
                indexPairs = matchFeatures(features1, features2, ...
                    'Unique', true, ...           % Each feature matches at most once
                    'MaxRatio', 0.75, ...         % Lowe's ratio test threshold
                    'MatchThreshold', 10.0);      % Maximum distance threshold
            catch
                % Fallback for older MATLAB versions without ratio test
                indexPairs = matchFeatures(features1, features2, 'Unique', true);
            end
            
            stats.numMatches = size(indexPairs, 1);
            
            % Need minimum 4 correspondences to compute homography
            % (8 degrees of freedom, each point gives 2 constraints)
            if size(indexPairs, 1) < 4
                matches = [];
                stats.numInliers = 0;
                stats.inlierRatio = 0;
                stats.H = eye(3);
                return;
            end
            
            % Extract the matched point coordinates
            matchedPoints1 = points1(indexPairs(:, 1), :);
            matchedPoints2 = points2(indexPairs(:, 2), :);
            
            % Format matches as [x1, y1, x2, y2] for each correspondence
            matches = [matchedPoints1, matchedPoints2];
            
            % Estimate homography using RANSAC to handle outliers
            % RANSAC iteratively finds the best transformation by sampling
            [H, inliers] = obj.estimateHomography(matchedPoints1, matchedPoints2);
            
            stats.H = H;
            stats.numInliers = sum(inliers);
            stats.inlierRatio = sum(inliers) / size(matches, 1);
            
            % Keep only the inlier matches
            matches = matches(inliers, :);
        end
        
        function [H, inliers] = estimateHomography(obj, points1, points2)
            % Robustly estimate homography using RANSAC
            % RANSAC handles outliers by finding the transformation that
            % fits the most points within a threshold
            
            pts1 = double(points1);
            pts2 = double(points2);
            
            try
                % Use MATLAB's built-in RANSAC implementation
                [tform, inlierIdx] = estimateGeometricTransform2D(...
                    pts2, pts1, 'projective', ...
                    'MaxNumTrials', obj.config.ransacMaxTrials, ...
                    'Confidence', obj.config.ransacConfidence, ...
                    'MaxDistance', 3.0);  % 3 pixel reprojection error threshold
                
                % Extract homography matrix (note the transpose)
                H = tform.T';
                inliers = inlierIdx;
                
            catch ME
                % Try older MATLAB function name
                try
                    [tform, inlierIdx] = estimateGeometricTransform(...
                        pts2, pts1, 'projective', ...
                        'MaxNumTrials', obj.config.ransacMaxTrials, ...
                        'Confidence', obj.config.ransacConfidence);
                    
                    H = tform.T';
                    inliers = inlierIdx;
                    
                catch
                    % Last resort: implement RANSAC myself
                    [H, inliers] = obj.ransacHomography(pts1, pts2);
                end
            end
            
            % Verify the homography is numerically stable
            if ~obj.isValidHomography(H)
                warning('Degenerate homography detected, using identity');
                H = eye(3);
                inliers = false(size(points1, 1), 1);
            end
        end
        
        function valid = isValidHomography(obj, H)
            % Check if homography matrix is valid (non-degenerate)
            % A degenerate homography can't be used for warping
            
            % Check determinant (should be non-zero and reasonable)
            detH = det(H);
            if abs(detH) < 0.01 || abs(detH) > 100
                valid = false;
                return;
            end
            
            % Check condition number (numerical stability)
            % High condition number means the matrix is nearly singular
            if cond(H) > 1e6
                valid = false;
                return;
            end
            
            % Check for NaN or Inf values
            if any(~isfinite(H(:)))
                valid = false;
                return;
            end
            
            valid = true;
        end
        
        function [H, inliers] = ransacHomography(obj, pts1, pts2)
            % Manual RANSAC implementation for homography estimation
            % This is a fallback if MATLAB's built-in functions aren't available
            
            numPoints = size(pts1, 1);
            maxTrials = obj.config.ransacMaxTrials;
            threshold = 3.0;  % Pixel error threshold
            
            bestH = eye(3);
            bestInliers = false(numPoints, 1);
            bestNumInliers = 0;
            
            % RANSAC main loop
            for trial = 1:maxTrials
                % Randomly sample 4 point correspondences
                % 4 points define a homography (8 DOF)
                sample = randperm(numPoints, 4);
                
                % Compute homography from this minimal set
                H_sample = obj.computeHomographyDLT(pts1(sample, :), pts2(sample, :));
                
                if ~obj.isValidHomography(H_sample)
                    continue;  % Skip degenerate cases
                end
                
                % Test all points against this homography
                % Transform points from image 1 to image 2
                pts1_h = [pts1, ones(numPoints, 1)]';  % Homogeneous coordinates
                pts2_proj = H_sample * pts1_h;
                pts2_proj = pts2_proj(1:2, :) ./ pts2_proj(3, :);  % Normalize
                
                % Compute reprojection errors
                errors = sqrt(sum((pts2' - pts2_proj).^2, 1));
                inliers_trial = errors < threshold;
                numInliers = sum(inliers_trial);
                
                % Keep best model so far
                if numInliers > bestNumInliers
                    bestNumInliers = numInliers;
                    bestInliers = inliers_trial';
                    bestH = H_sample;
                end
                
                % Early termination if we found a really good model
                if numInliers > 0.9 * numPoints
                    break;
                end
            end
            
            H = bestH;
            inliers = bestInliers;
        end
        
        function H = computeHomographyDLT(obj, pts1, pts2)
            % Compute homography using Direct Linear Transform (DLT)
            % This solves for H in the equation: pts2 = H * pts1
            % using linear least squares
            
            numPoints = size(pts1, 1);
            
            % Build the constraint matrix A
            % Each point correspondence gives 2 equations
            A = zeros(2 * numPoints, 9);
            for i = 1:numPoints
                x1 = pts1(i, 1);
                y1 = pts1(i, 2);
                x2 = pts2(i, 1);
                y2 = pts2(i, 2);
                
                % These equations come from the cross product
                % [x2, y2, 1]' × H[x1, y1, 1]' = 0
                A(2*i-1, :) = [-x1, -y1, -1, 0, 0, 0, x2*x1, x2*y1, x2];
                A(2*i, :) = [0, 0, 0, -x1, -y1, -1, y2*x1, y2*y1, y2];
            end
            
            % Solve Ah = 0 using SVD
            % The solution is the eigenvector with smallest eigenvalue
            [~, ~, V] = svd(A);
            h = V(:, 9);  % Last column corresponds to smallest singular value
            
            % Reshape from 9x1 vector to 3x3 matrix
            H = reshape(h, 3, 3)';
            
            % Normalize so H(3,3) = 1 (remove scale ambiguity)
            H = H / H(3, 3);
        end
    end
end