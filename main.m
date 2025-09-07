%% Panorama Stitcher 
% Image stitching using custom FAST detection and homography estimation

clear; close all; clc;

% Add utility functions to path
addpath('utils');

%% Configuration Setup
config = struct(); 

% Define image sets for processing
% These are photos I took around SFU campus - the concrete brutalist 
% architecture provides excellent corner features for matching
config.imageSets = {
    {'demo_images/set1_img1.png', 'demo_images/set1_img2.png'};  % WAC Bennett Library
    {'demo_images/set2_img1.png', 'demo_images/set2_img2.png', ...
     'demo_images/set2_img3.png', 'demo_images/set2_img4.png'};  % Academic Quadrangle 
    {'demo_images/set3_img1.png', 'demo_images/set3_img2.png', ...
     'demo_images/set3_img3.png', 'demo_images/set3_img4.png'};  % Reflection pond area
};

% Algorithm parameters - tuned through experimentation
config.detector = 'FASTR';        % Using robust FAST with Harris filtering
config.descriptor = 'SURF';       % SURF provides rotation invariance
config.blendMethod = 'linear';    % Linear feathering for smooth transitions

% Output configuration
config.outputDir = 'output';
config.saveVisualizations = true;
config.verbose = true;

% FAST detector parameters
% Threshold of 0.15 means a pixel must be 15% brighter/darker than center
% to be considered part of the corner arc
config.fastThreshold = 0.15;
config.fastN = 12;  % Require 12 contiguous pixels (75% of circle)

% Harris filtering removes ambiguous corners
% Lower threshold = more selective (removes weak corners)
config.harrisThreshold = 0.005;  

% RANSAC parameters for robust homography estimation
config.ransacMaxTrials = 500;     % Max iterations to find best model
config.ransacConfidence = 99.9;   % Statistical confidence level

% Enable feature matching visualization
config.createMatchViz = true;

%% Initialize Processing
fprintf('Panorama Stitcher - Computer Vision Pipeline\n');
fprintf('Processing %d image sets from campus\n\n', length(config.imageSets));

% Create output directory if it doesn't exist
if ~exist(config.outputDir, 'dir')
    mkdir(config.outputDir);
end

% Initialize the main stitching pipeline
stitcher = PanoramaStitcher(config);

%% Main Processing Loop
results = cell(length(config.imageSets), 1);

for setIdx = 1:length(config.imageSets)
    fprintf('Processing Set %d (%d images)...\n', ...
            setIdx, length(config.imageSets{setIdx}));
    
    try
        % For 2-image panoramas, I generate matching visualizations
        % to demonstrate the quality of feature correspondence
        if length(config.imageSets{setIdx}) == 2 && config.createMatchViz
            fprintf('  Generating feature match visualization\n');
            createMatchVisualizationForPair(config.imageSets{setIdx}, ...
                                           stitcher, config, setIdx);
        end
        
        % Execute the main stitching pipeline
        tic;
        [panorama, stats] = stitcher.stitchImages(config.imageSets{setIdx});
        elapsedTime = toc;
        
        % Save the resulting panorama
        outputPath = fullfile(config.outputDir, sprintf('panorama_set%d.jpg', setIdx));
        imwrite(panorama, outputPath);
        fprintf('  Panorama saved: %s\n', outputPath);
        
        % Store processing results
        results{setIdx} = struct(...
            'success', true, ...
            'panorama', panorama, ...
            'stats', stats, ...
            'time', elapsedTime, ...
            'outputPath', outputPath ...
        );
        
        % Display processing statistics
        if config.verbose
            fprintf('  Detected features: %d average\n', stats.avgFeatures);
            fprintf('  Matched pairs: %d average\n', stats.avgMatches);
            fprintf('  RANSAC inliers: %.1f%%\n', stats.avgInlierRatio * 100);
            fprintf('  Processing time: %.2f seconds\n', elapsedTime);
        end
        
    catch ME
        fprintf('  Error in set %d: %s\n', setIdx, ME.message);
        results{setIdx} = struct('success', false, 'error', ME.message);
    end
    fprintf('\n');
end

%% Generate Processing Report
fprintf('Final Summary\n');
fprintf('-------------\n');

successCount = sum(cellfun(@(r) r.success, results));
fprintf('Successfully processed: %d/%d panoramas\n', ...
        successCount, length(results));

totalTime = sum(cellfun(@(r) r.time, results(cellfun(@(r) r.success, results))));
fprintf('Total processing time: %.2f seconds\n', totalTime);

if successCount > 0
    avgTime = totalTime / successCount;
    fprintf('Average time per panorama: %.2f seconds\n', avgTime);
end

% Write summary to file for documentation
summaryPath = fullfile(config.outputDir, 'processing_summary.txt');
fid = fopen(summaryPath, 'w');
fprintf(fid, 'Panorama Processing Report\n');
fprintf(fid, 'SFU Campus Image Stitching\n\n');
fprintf(fid, 'Date: %s\n', datestr(now));
fprintf(fid, 'Detector: %s\n', config.detector);
fprintf(fid, 'Descriptor: %s\n', config.descriptor);
fprintf(fid, 'Blend Method: %s\n\n', config.blendMethod);
fprintf(fid, 'Results:\n');
for i = 1:length(results)
    if results{i}.success
        fprintf(fid, 'Set %d: Completed in %.2fs (%d images)\n', ...
                i, results{i}.time, length(config.imageSets{i}));
    else
        fprintf(fid, 'Set %d: Failed - %s\n', i, results{i}.error);
    end
end
fclose(fid);

fprintf('\nProcessing complete. Results saved to %s\n', config.outputDir);

%% Feature Matching Visualization Function
function createMatchVisualizationForPair(imagePaths, stitcher, config, setIdx)
    % This function creates visualizations showing how features are matched
    % between image pairs, demonstrating the effectiveness of FAST detection
    % combined with SURF descriptors
    
    % Load the image pair
    img1 = imread(imagePaths{1});
    img2 = imread(imagePaths{2});
    img1 = im2double(img1);
    img2 = im2double(img2);
    
    % Convert to grayscale for feature detection
    gray1 = rgb2gray(img1);
    gray2 = rgb2gray(img2);
    
    % Detect FASTR features (FAST with Harris filtering)
    [points1, features1] = stitcher.detectAndDescribe(gray1);
    [points2, features2] = stitcher.detectAndDescribe(gray2);
    
    % Match features using SURF descriptors
    matcher = FeatureMatcher(config);
    [matches, matchStats] = matcher.matchFeatures(features1, features2, points1, points2);
    
    if ~isempty(matches)
        % Create visualization showing matched features with connecting lines
        fig = figure('Visible', 'off', 'Position', [100, 100, 1400, 600]);
        showMatchedFeatures(img1, img2, matches(:,1:2), matches(:,3:4), 'montage');
        title(sprintf('FASTR Feature Matches: %d initial, %d inliers (%.1f%% ratio)', ...
               matchStats.numMatches, matchStats.numInliers, matchStats.inlierRatio * 100));
        saveas(fig, fullfile(config.outputDir, sprintf('set%d_fastr_matches.png', setIdx)));
        close(fig);
        
        % For the first two sets, also show FAST without Harris filtering
        % This demonstrates the improvement from Harris corner response
        if setIdx <= 2
            tempConfig = config;
            tempConfig.detector = 'FAST';
            fastDetector = FastDetector(tempConfig);
            
            % Detect raw FAST features without Harris filtering
            pointsFAST1 = fastDetector.detectFAST(gray1);
            pointsFAST2 = fastDetector.detectFAST(gray2);
            
            if ~isempty(pointsFAST1) && ~isempty(pointsFAST2)
                cornerPts1 = cornerPoints(pointsFAST1);
                cornerPts2 = cornerPoints(pointsFAST2);
                [featuresFAST1, validPts1] = extractFeatures(gray1, cornerPts1, 'Method', config.descriptor);
                [featuresFAST2, validPts2] = extractFeatures(gray2, cornerPts2, 'Method', config.descriptor);
                
                % Match and visualize
                [matchesFAST, statsFAST] = matcher.matchFeatures(featuresFAST1, featuresFAST2, ...
                                                                  validPts1.Location, validPts2.Location);
                
                if ~isempty(matchesFAST)
                    fig = figure('Visible', 'off', 'Position', [100, 100, 1400, 600]);
                    showMatchedFeatures(img1, img2, matchesFAST(:,1:2), matchesFAST(:,3:4), 'montage');
                    title(sprintf('FAST Feature Matches (no Harris): %d initial, %d inliers (%.1f%% ratio)', ...
                           statsFAST.numMatches, statsFAST.numInliers, statsFAST.inlierRatio * 100));
                    saveas(fig, fullfile(config.outputDir, sprintf('set%d_fast_matches.png', setIdx)));
                    close(fig);
                end
            end
        end
        
        fprintf('  Feature matching visualizations created\n');
    end
end