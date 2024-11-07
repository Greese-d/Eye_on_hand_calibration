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
%joint_target = calibration.qs_calib(10, :);
%dobot.PublishTargetJoint(joint_target);
%calibration.moveToTargetQs(calibration.qs_calib(1, :))

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
%newlocation = lastlocation * cameraToEndEffectorTform.A * target
newlocation = lastlocation * transl(cameraToEndEffectorTform.Translation)
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
    calibration.moveToTargetQs(calibration.qs_calib(i, :))
    pause(2)
end


%% save waypoints

% Specify number of pictures
num_of_pic = 30;

for i = 1:1:num_of_pic
    pause()
    calibration = calibration.saveWaypoint('calib5');
    disp("Taken a picture No" + i)
end
disp("finished taking pictures")


%% Take pictures for camera calibration
num_of_pics = 20;
for i = 1:1:num_of_pics
    pause()
    calibration.takePicture('calib6');
    disp("Taken a picture No" + i)
end
disp("finished taking pictures")

%% Load q values:
calibration.qs_calib = [0.0166    0.1010    0.0897         0
   -0.0079    0.3586    0.0663         0
   -0.0037    0.6034    0.0683         0
   -0.0097    0.5936    0.4382         0
   -0.0115    0.3991    0.4892         0
   -0.0102    0.3737    0.5514         0
    0.0111    0.6791    0.7031         0
    0.0185    0.2897   -0.2456         0
    0.0282    0.0297   -0.2106         0
   -0.0102    0.4303   -0.2415         0
   -0.0111    0.5418   -0.0824         0
   -0.0360    0.6670   -0.2090         0
    0.1659    0.6289   -0.1830         0
    0.2273    0.4372   -0.0439         0
    0.1954    0.1301    0.0227         0
    0.1783    0.2058   -0.1771         0
    0.1289    0.4112    0.4987         0
   -0.1488    0.5944   -0.2587         0
   -0.2097    0.3451   -0.2612         0
   -0.2389    0.0612   -0.2380         0
   -0.0864    0.3095    0.3260         0
   -0.0480    0.4659    0.5527         0
   -0.2208    0.3999   -0.2455         0
   -0.1746    0.5666   -0.1071         0
   -0.0767    0.5660    0.4218         0
   -0.1732   -0.0015   -0.0339         0
   -0.1506    0.2257   -0.2267         0
   -0.1372    0.4394   -0.1697         0
   -0.0730    0.4544   -0.2328         0];

%% Getting data for computation
boardToCameraTform = getExtrinsics("robotCalib");
endEffectorToBaseTform = getKinematics(dobot, calibration.qs_calib);

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
