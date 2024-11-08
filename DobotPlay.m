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
run('rvctools/startup_rvc.m')

%% Get calibration data
calibration.getData(15, 'calib5')   % Specify number of pictures

%% Load q values:
% If skipped last step
%Below are values corresponding to robotCalib set of pictures

new_q = [0.0166    0.1010    0.0897         0
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
   -0.0730    0.4544   -0.2328         0]

calibration = calibration.loadQs(new_q);


%% Computation

calibration = calibration.computeExtrinsics('robotCalib');
calibration = calibration.computeKinematics;
calibration = calibration.computeFinalTform;

%% Load camera tr
% If skipped last step
newTr = [     0    1.0000         0    0.0614
                     1.0000         0         0         0
                          0         0   -1.0000    0.1050
                          0         0         0    1.0000];
calibration.cameraToEndEffectorTform = rigidtform3d(newTr);



%% Specify target location relative to end effector
target = transl(0, -0.02, 0);
lastlocation = dobot.getCurrentEndEffectorPose
newlocation = lastlocation.A * target

%% Specify target in camera's coordinate system
target = transl(0, 0, 0.02);
lastlocation = dobot.getCurrentEndEffectorPose
newlocation = lastlocation.A * calibration.cameraToEndEffectorTform.A * target * inv(calibration.cameraToEndEffectorTform.A)
%newlocation = lastlocation.A * transl(calibration.cameraToEndEffectorTform.Translation)


%% Tesing calibration
testImage = imread("object_test/image_10.jpg");
intrinsics = getIntrinsics;
undistortedTestImage = undistortImage(testImage, intrinsics);

% Specify the tag family and tag size of the AprilTag.
tagFamily = 'tag36h11';
tagSize = 0.019; % AprilTag size in meters

% Detect AprilTag in test image.
[~,~,aprilTagToCameraTform] = readAprilTag(undistortedTestImage,tagFamily,intrinsics,tagSize);

%% Calculate desired transform
lastlocation = dobot.getCurrentEndEffectorPose
cameraToBaseTr = lastlocation.A * calibration.cameraToEndEffectorTform.A;


% Find the transformation from the robot base to the April Tag.
newlocation = cameraToBaseTr * aprilTagToCameraTform.A;
targetPosition = tagToBaseTr(1:3,4)'


%% Move to target location
rotationMatrix = newlocation(1:3, 1:3)
translationVector = newlocation(1:3, 4)'
dobot.PublishEndEffectorPose(translationVector, rotationMatrix);