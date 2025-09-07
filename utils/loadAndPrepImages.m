function images = loadAndPrepImages(imagePaths)
    % LOADANDPREPIMAGES - Load and prepare images for stitching
    %
    % Input:
    %   imagePaths - Cell array of image file paths
    %
    % Output:
    %   images - Cell array of loaded and prepared images
    
    numImages = length(imagePaths);
    images = cell(numImages, 1);
    
    for i = 1:numImages
        % Check if file exists
        if ~exist(imagePaths{i}, 'file')
            error('Image file not found: %s', imagePaths{i});
        end
        
        % Load image
        try
            img = imread(imagePaths{i});
        catch ME
            error('Failed to read image %s: %s', imagePaths{i}, ME.message);
        end
        
        % Convert to double
        img = im2double(img);
        
        % Ensure RGB (convert grayscale to RGB if needed)
        if size(img, 3) == 1
            img = repmat(img, [1, 1, 3]);
        elseif size(img, 3) == 4
            % Remove alpha channel if present
            img = img(:, :, 1:3);
        elseif size(img, 3) ~= 3
            error('Unsupported image format for %s', imagePaths{i});
        end
        
        % Resize if too large (max dimension 750 pixels)
        maxDim = max(size(img, 1), size(img, 2));
        if maxDim > 750
            scale = 750 / maxDim;
            img = imresize(img, scale, 'bilinear');
            fprintf('  Resized image %d: %.0fx%.0f -> %.0fx%.0f\n', ...
                    i, maxDim, maxDim/size(img,1)*size(img,2), ...
                    size(img, 1), size(img, 2));
        end
        
        % Check for valid values
        if any(~isfinite(img(:)))
            error('Image %s contains invalid values (NaN or Inf)', imagePaths{i});
        end
        
        % Store prepared image
        images{i} = img;
    end
    
    % Verify all images are loaded
    if numImages ~= length(images)
        error('Failed to load all images');
    end
    
    fprintf('  Loaded %d images successfully\n', numImages);
end