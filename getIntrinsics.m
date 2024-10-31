function intrinsics = getIntrinsics()
    % Path to your images
    imageFolder = fullfile("photos2"); % Adjust the path if "photos" is not in the current directory

    % Load images from the folder
    images = imageDatastore(imageFolder);

    % Detect the checkerboard corners in the images
    [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

    % Generate the world coordinates of the checkerboard corners in the 
    % pattern-centric coordinate system, with the upper-left corner at (0,0). 
    % Adjust the square size if needed.
    squareSize = 29; % in millimeters
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Calibrate the camera
    I = readimage(images, 1);
    imageSize = [size(I, 1), size(I, 2)];
    cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
        ImageSize=imageSize);
    intrinsics = cameraParams.Intrinsics;
end
