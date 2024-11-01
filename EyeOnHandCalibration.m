classdef EyeOnHandCalibration 
    properties
        dobot
        camera
        intrinsics
        qs_calib
    end

    methods
        function obj = EyeOnHandCalibration(dobot, camera)
            obj.dobot = dobot;
            obj.camera = camera;
            obj.qs_calib = [];
        end

        function moveToTargetQs(obj, targetQDeg)
            target_deg = targetQDeg;
            joint_target = deg2rad(target_deg);
            obj.dobot.PublishTargetJoint(joint_target);
        end

        function obj = memoriseQs(obj)
            q_mem = obj.dobot.GetCurrentJointState();
            obj.qs_calib(end+1, :) = q_mem;
            %obj.qs_calib = [obj.qs_calib; q_mem];  % Append to qs_calib without overwriting
        end

        function takePicture(obj, foldername)
            % Directory to save images
            outputFolder = foldername;

            if ~exist(outputFolder, 'dir')
                mkdir(outputFolder);  % Create directory if it doesn't exist
            end

            % Dynamically determine the next available image number
            imageFiles = dir(fullfile(outputFolder, 'image_*.jpg'));
            picNo = length(imageFiles) + 1;  % Increment based on existing files

            % Capture and display the image
            img = snapshot(obj.camera);
            imshow(img);
            title(['Image ', num2str(picNo)]);
            drawnow;

            % Save the image with a unique filename
            filename = fullfile(outputFolder, sprintf('image_%02d.jpg', picNo));
            imwrite(img, filename);
        end

        function obj = saveWaypoint(obj, foldername)
            obj = obj.memoriseQs();       % Memorize the current joint state
            obj.takePicture(foldername);             % Take and save the picture
        end

    end
end