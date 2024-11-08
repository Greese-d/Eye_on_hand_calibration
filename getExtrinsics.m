function [boardToCameraTform] = getExtrinsics(intrinsicsFolder, foldername)

disp("Getting intrinsics")
% Load the images
images = imageDatastore(intrinsicsFolder);

% Detect the checkerboard corners in the images
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Generate the world coordinates of the checkerboard corners
% with the upper-left corner at (0,0)
squareSize = 0.012; % in m
%boardSize= [5 6];
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

% Calibrate the camera
I = readimage(images, 1);
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    ImageSize=imageSize);
intrinsics = cameraParams.Intrinsics;
disp("Obtained intrinsics for the camera")

figure;
hold on;
axis equal;


imagesRobot = imageDatastore(foldername);
noOfPics = numel(imagesRobot.Files);
boardToCameraTform = rigidtform3d.empty(noOfPics, 0);
for i=1:1:noOfPics
    % Load an image at a new location
    imOrig = readimage(imagesRobot, i);

    imEnhanced = imadjust(rgb2gray(imOrig));
    imSharpened = imsharpen(imEnhanced, 'Radius', 2, 'Amount', 1.5);
    imFiltered = imgaussfilt(imSharpened, 2);

    % Display the input image
    %figure;
    %imshow(imOrig);
    %title("Input Image");

    % Undistort the image
    [im, newIntrinsics] = undistortImage(imFiltered, intrinsics, OutputView="full");

    % Find the reference object in the new image
    [imagePoints, boardSize] = detectCheckerboardPoints(im);

    % Compensate for the image coordinate system shift
    newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
    imagePoints = imagePoints + newOrigin;


    % Check corner detection
    % imshow(im);
    % hold on
    % plot(imagePoints(:,1), imagePoints(:,2), 'ro');
    % title(['Checkerboard Corners for Image ', num2str(i)]);
    % hold off
    % pause(2);  % Pause to inspect each image

    % Calculate new extrinsics
    disp("Estimating extrinsics for picture No" + i)
    camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);

    % Calculate the camera pose
    camPose = extr2pose(camExtrinsics);
  
    boardToCameraTform(i) = camExtrinsics;

    % Visualize the camera pose and world points
    plotCamera(AbsolutePose=camPose, Size=0.005);
    %hold on;
    pcshow([worldPoints, zeros(size(worldPoints, 1), 1)], ...
        VerticalAxisDir="down", MarkerSize=2);
    %hold off;

end
end