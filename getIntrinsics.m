function intrinsics = getIntrinsics()
% Create a set of calibration images. CHANGE TO VALID LOCATION
images = imageDatastore(fullfile(toolboxdir("vision"),"visiondata", ...
      "calibration","slr"));

% I = readimage(images, 1);  % Reads the first image in the datastore
% imshow(I);  

% Detect the checkerboard corners in the images.
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

% Generate the world coordinates of the checkerboard corners in the 
% pattern-centric coordinate system, with the upper-left corner at (0,0). 
% The square size is in millimeters.
squareSize = 29;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

% Calibrate the camera.
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
cameraParams = estimateCameraParameters(imagePoints,worldPoints, ...
    ImageSize=imageSize);
intrinsics = cameraParams.Intrinsics;
end

