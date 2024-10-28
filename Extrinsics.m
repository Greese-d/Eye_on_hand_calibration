%% This is example code from MATLAB on how to use estimateExtrinsics
% Accessed using "openExample('vision/CalculateCameraExtrinsicsExample')" 
% in the terminal

% Camera's intrinsics
intrinsics = getIntrinsics;

% Load an image at a new location. CHANGE TO VALID LOCATION
imOrig = readimage(images,9); 
figure 
imshow(imOrig)
title("Input Image")

% Undistort the image.
[im,newIntrinsics] = undistortImage(imOrig,intrinsics,OutputView="full");

% Find the reference object in the new image.
[imagePoints,boardSize] = detectCheckerboardPoints(im);

% Compensate for the image coordinate system shift.
newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints+newOrigin;

% Calculate new extrinsics.
camExtrinsics = estimateExtrinsics(imagePoints,worldPoints,newIntrinsics);

% Calculate the camera pose.
camPose = extr2pose(camExtrinsics);

figure
plotCamera(AbsolutePose=camPose,Size=20);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  VerticalAxisDir="down",MarkerSize=40);



