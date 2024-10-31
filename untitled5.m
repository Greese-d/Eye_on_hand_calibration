% Load the images
images = imageDatastore("photos");

% Detect the checkerboard corners in the images
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Generate the world coordinates of the checkerboard corners
% with the upper-left corner at (0,0)
squareSize = 29; % in mm
%boardSize= [5 6];
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

% Calibrate the camera
I = readimage(images, 1); 
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    ImageSize=imageSize);
intrinsics = cameraParams.Intrinsics;

% Load an image at a new location
imOrig = readimage(images, 9); 

% Display the input image
figure;
imshow(imOrig);
title("Input Image");

% Undistort the image
[im, newIntrinsics] = undistortImage(imOrig, intrinsics, OutputView="full");

% Find the reference object in the new image
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Compensate for the image coordinate system shift
newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints + newOrigin;

% Calculate new extrinsics
camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);

% Calculate the camera pose
camPose = extr2pose(camExtrinsics);

% Visualize the camera pose and world points
figure;
plotCamera(AbsolutePose=camPose, Size=20);
hold on;
pcshow([worldPoints, zeros(size(worldPoints, 1), 1)], ...
    VerticalAxisDir="down", MarkerSize=40);
hold off;