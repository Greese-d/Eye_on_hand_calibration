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

%% Initiate calibratio position array
qs_calib = [];

%% Get current qvalues
dobot.GetCurrentJointState

%% Get current end-effector pose
T = dobot.PublishEndEffectorPose

%% Move to targer Q's

%target_deg = [0 20 -10 0];
%joint_target = deg2rad(target_deg);
joint_target = [-0.0605    0.1299    0.3203         0]
dobot.PublishTargetJoint(joint_target);

%% Move to memory Q's

joint_target = q_mem;
dobot.PublishTargetJoint(joint_target);

%% Memorise Q's
q_mem = dobot.GetCurrentJointState;

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

%% Add qs to calibration qs array

q_mem = dobot.GetCurrentJointState;
qs_calib(end+1, :) = q_mem

%% Set up camera

camera = webcam("HP 320 FHD Webcam");

%% Move through qs
for i = 1:1:15
    calibration.moveToTargetQs(rad2deg(calibration.qs_calib(i, :)))
    pause(2)
end


%% save waypoint
%saveWaypoint();
for i = 1:1:10
    pause()
    calibration = calibration.saveWaypoint('calib2');
    disp("Taken a picture No" + i)
end
disp("finished taking pictures")

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
