function saveResults(panorama, outputPath, stats)
    % SAVERESULTS - Save panorama and optional statistics
    %
    % Inputs:
    %   panorama - The stitched panorama image
    %   outputPath - Path where to save the image
    %   stats - Optional statistics structure
    
    % Ensure output directory exists
    [outputDir, ~, ~] = fileparts(outputPath);
    if ~isempty(outputDir) && ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
    
    % Ensure valid image range [0, 1]
    panorama = max(0, min(1, panorama));
    
    % Save image
    try
        imwrite(panorama, outputPath);
        fprintf('  Saved panorama: %s\n', outputPath);
    catch ME
        error('Failed to save panorama: %s', ME.message);
    end
    
    % Save statistics if provided
    if nargin >= 3 && ~isempty(stats)
        % Create stats filename
        [path, name, ~] = fileparts(outputPath);
        statsPath = fullfile(path, [name '_stats.mat']);
        
        % Save stats
        try
            save(statsPath, 'stats');
            fprintf('  Saved statistics: %s\n', statsPath);
        catch
            warning('Could not save statistics file');
        end
    end
end