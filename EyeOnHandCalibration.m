classdef EyeOnHandCalibration 
    properties
        dobot
        camera
        intrinsics
        boardToCameraTform
        endEffectorToBaseTform
        cameraToEndEffectorTform
        qs_calib
    end

    methods
        function obj = EyeOnHandCalibration(dobot, camera)
            obj.dobot = dobot;
            obj.camera = camera;
            obj.qs_calib = [];
        end

        function moveToTargetQs(obj, targetQ)
            obj.dobot.PublishTargetJoint(targetQ);
        end

        function obj = memoriseQs(obj)
            q_mem = obj.dobot.GetCurrentJointState();
            obj.qs_calib(end+1, :) = q_mem;
        end

        
        function takePicture(obj, outputFolder)
            % Directory to save images

            if ~exist(outputFolder, 'dir')
                mkdir(outputFolder);  % Create directory if it doesn't exist
            end

            % Dynamically determine the next available image number
            imageFiles = dir(fullfile(outputFolder, 'image_*.jpg'));
            picNo = length(imageFiles) + 1;  % Increment based on existing files

            % Capture and display the image
            img = snapshot(obj.camera);
            %imshow(img);
            %title(['Image ', num2str(picNo)]);
            %drawnow;

            % Save the image with a unique filename
            filename = fullfile(outputFolder, sprintf('image_%02d.jpg', picNo));
            imwrite(img, filename);
        end


        function obj = saveWaypoint(obj, foldername)
            obj = obj.memoriseQs();       % Memorize the current joint state
            obj.takePicture(foldername);             % Take and save the picture
        end


        function obj = getData(obj, noOfPoses, outputFolder)
            for i = 1:1:noOfPoses
                pause()
                obj = obj.saveWaypoint(outputFolder);
                disp("Saved a waypoint No" + i)
            end
            disp("Finished getting waypoints")
        end


        function obj = loadQs(obj, qArray)
            obj.qs_calib = qArray;
        end


        function obj = resetQs(obj)
            obj.qs_calib = [];
        end


        function obj = computeExtrinsics(obj, inputFolder)
            obj.boardToCameraTform = getExtrinsics(inputFolder);
        end


        function obj = computeKinematics(obj)
            obj.endEffectorToBaseTform = getKinematics(obj.dobot, obj.qs_calib);
        end


        function obj = computeFinalTform(obj)
            obj.cameraToEndEffectorTform = helperEstimateHandEyeTransform(obj.boardToCameraTform, obj.endEffectorToBaseTform, "eye-in-hand");
           
            obj.cameraToEndEffectorTform.A(2, 4) = 0; %Camera is alligned on y axis
            rotation = obj.cameraToEndEffectorTform.A(1:3, 1:3);
            obj.cameraToEndEffectorTform.R = eye(3) * rotx(pi) * rotz(-pi/2);
            obj.cameraToEndEffectorTform.Translation(3) = 0.09; 


            % angles = rotm2eul(rotation, 'XYZ');
            % disp(rad2deg(angles));
            %obj.cameraToEndEffectorTform.R = eye(3) * rotx(deg2rad(-167));
            %obj.cameraToEndEffectorTform.A = obj.cameraToEndEffectorTform.A * trotz(deg2rad(144))
            %obj.cameraToEndEffectorTform = helperEstimateHandEyeTransform4DOF(obj.boardToCameraTform, obj.endEffectorToBaseTform);

        end


        function testRobotOn(obj)
            % Turn on gripper
            onOff = 1;
            openClose = 1;
            obj.dobot.PublishToolState(onOff,openClose);

            % Open gripper
            onOff = 1;
            openClose = 0;
            obj.dobot.PublishToolState(onOff,openClose);
            pause(1)

            % Close gripper
            onOff = 1;
            openClose = 1;
            obj.dobot.PublishToolState(onOff,openClose);
            pause(1)

            % Turn off gripper
            onOff = 0;
            openClose = 0;
            obj.dobot.PublishToolState(onOff,openClose);

            disp("Current joint state: ")
            obj.dobot.GetCurrentJointState

            disp("Current end-effector pose: ")
            obj.dobot.getCurrentEndEffectorPose

        end

        function moveThroughQs(obj)
            for i = 1:1:size(obj.qs_calib, 1)
                obj.moveToTargetQs(obj.qs_calib(i, :))
                pause(2)
            end
        end


    end
end