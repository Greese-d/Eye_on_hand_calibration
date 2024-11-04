%% Setup
clear all;
clc;
close all;
rosshutdown;
pause(1)
rosinit('192.168.27.1')
dobot = DobotMagician();
camera = webcam("HP 320 FHD Webcam");
calibration = EyeOnHandCalibration(dobot, camera);

%% Get current qvalues
dobot.GetCurrentJointState

%% Get current end-effector pose
tr = dobot.getCurrentEndEffectorPose

%% Move to targer Q's

%target_deg = [0 20 -10 0];
%joint_target = deg2rad(target_deg);
%joint_target = [-0.0605    0.1299    0.3203         0];
% join_target = calibration.qs_calib(10, :);
% dobot.PublishTargetJoint(joint_target);
calibration.moveToTargetQs(rad2deg(calibration.qs_calib(3, :)))

%% Specify target location relative to end effector
% translation = [0 0 -0.01];
% rotation = eye(3, 3);
target = transl(0, 0.02, 0);
lastlocation = dobot.getCurrentEndEffectorPose
newlocation = lastlocation * target

%% Move to target location
rotationMatrix = newlocation(1:3, 1:3)
translationVector = newlocation(1:3, 4)'
dobot.PublishEndEffectorPose(translationVector, rotationMatrix);

%% Specify target in camera's coordinate system
target = transl(0, 0, 0.02);
lastlocation = dobot.getCurrentEndEffectorPose
newlocation = lastlocation * cameraToEndEffectorTform.A * target

%% 
cameraToEndEffectorTform.Translation = cameraToEndEffectorTform.Translation * 0.1
%% Turn on gripper
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);

%% Close gripper
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);

%% Open gripper
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Turn off gripper
onOff = 0;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Move through qs
for i = 1:1:size(calibration.qs_calib, 1)
    calibration.moveToTargetQs(rad2deg(calibration.qs_calib(i, :)))
    pause(2)
end


%% save waypoints

% Specify number of pictures
num_of_pic = 15;

for i = 1:1:num_of_pic
    pause()
    calibration = calibration.saveWaypoint('calib4');
    disp("Taken a picture No" + i)
end
disp("finished taking pictures")


%% Load q values:
calibration.qs_calib = [

   -0.0605    0.1299    0.3203         0
    0.0282    0.1283    0.2466         0
    0.0032    0.3936    0.2448         0
   -0.1363    0.3823    0.1944         0
   -0.2758    0.1524    0.1019         0
   -0.1372    0.0365    0.1424         0
    0.0397    0.0347    0.1563         0
    0.0785    0.1764    0.1802         0
    0.0402    0.3697    0.2318         0
   -0.0092    0.4222    0.1827         0
   -0.0037    0.4203   -0.1533         0
   -0.0693    0.3420    0.5162         0
    0.0046    0.3434    0.5191         0
   -0.1866    0.3845    0.4712         0];

%% Getting data for computation
boardToCameraTform = Extrinsics();
endEffectorToBaseTform = Kinematics(dobot, calibration.qs_calib);

%% Camera to end-effector transform
cameraToEndEffectorTform = helperEstimateHandEyeTransform(boardToCameraTform, endEffectorToBaseTform, "eye-in-hand")

%% Tesing calibration
testImage = imread("textobj/image_01.jpg");
intrinsics = getIntrinsics;
undistortedTestImage = undistortImage(testImage, intrinsics);

% Specify the tag family and tag size of the AprilTag.
tagFamily = 'tag36h11';
tagSize = 0.019; % AprilTag size in meters

% Detect AprilTag in test image.
[~,~,aprilTagToCameraTform] = readAprilTag(undistortedTestImage,tagFamily,intrinsics,tagSize);
%% Moving to april tag

% Find the transformation from the robot base to the April Tag.
tagToEndEffectorTestTform = cameraToEndEffectorTform.A * aprilTagToCameraTform.A;
cubePosition = tagToEndEffectorTestTform(1:3,4)
grabbingSpotToEndEffectorTr = transl(0,0, 0.095) * tagToEndEffectorTestTform;

rotationMatrix = grabbingSpotToEndEffectorTr(1:3, 1:3);
translationVector = grabbingSpotToEndEffectorTr(1:3, 4)';
disp(grabbingSpotToEndEffectorTr)

dobot.PublishEndEffectorPose(translationVector, rotationMatrix)

%% Helper functions

% function moveToTargetQs(targetQDeg)
%     target_deg = targetQDeg;
%     joint_target = deg2rad(target_deg);
%     dobot.PublishTargetJoint(joint_target);
% end
% 
% function memoriseQs(dobot, qs_calib)
%     q_mem = dobot.GetCurrentJointState;
%     qs_calib(end+1, :) = q_mem;
% end
% 
% function takePicture()
% 
%     picNo = size(qs_calib, 1);
% 
%     % Directory to save images (optional)
%     outputFolder = 'eyeonhandcalib_images';
% 
%     if ~exist(outputFolder, 'dir')
%         mkdir(outputFolder);  % Create directory if it doesn't exist
%     end
% 
%     img = snapshot(camera);
% 
%     % Display the image
%     imshow(img);
%     title(['Image ', num2str(picNo)]);  % Display image count on title
%     drawnow;  % Refresh display immediately
% 
%     % Save the image with a unique filename
%     filename = fullfile(outputFolder, sprintf('image_%02d.jpg', picNo));
%     imwrite(img, filename);
% end
% 
% function saveWaypoint()
%     memoriseQs(dobot, qs_calib);
%     takePicture;
% end
