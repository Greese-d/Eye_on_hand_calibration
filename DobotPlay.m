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
joint_target = [-0.0605    0.1299    0.3203         0];
dobot.PublishTargetJoint(joint_target);

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
calibration.qs_calib = [-0.0675    0.4489    0.1725         0
   -0.0675    0.4489    0.1725         0
    0.0444    0.3180    0.2223         0
    0.0522    0.6266    0.2601         0
   -0.0305    0.6273    0.2761         0
   -0.0106    0.6278    0.0602         0
   -0.0176    0.4083    0.0823         0
   -0.0799    0.4210    0.0972         0
   -0.0808    0.6246    0.1211         0
    0.0582    0.4882    0.1340         0
   -0.0434    0.6131    0.1847         0
    0.0102    0.5259    0.0859         0
    0.1830    0.5283    0.0247         0
    0.2194    0.5243   -0.0824         0
    0.2583    0.5252   -0.0805         0
];

%% Getting data for computation
cameraToBoardTform = Extrinsics();
endEffectorToBaseTform = Kinematics(dobot, calibration.qs_calib);

%% Camera to end-effector transform
cameraToEndEffectorTform = helperEstimateHandEyeTransform(cameraToBoardTform, endEffectorToBaseTform, "eye-in-hand")

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
