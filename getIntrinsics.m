function intrinsics = getIntrinsics()
% Load the images
images = imageDatastore("calib2");

% Detect the checkerboard corners in the images
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Generate the world coordinates of the checkerboard corners
% with the upper-left corner at (0,0)
squareSize = 12; % in mm
%boardSize= [5 6];
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

% Calibrate the camera
I = readimage(images, 1);
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    ImageSize=imageSize);
intrinsics = cameraParams.Intrinsics;
end

