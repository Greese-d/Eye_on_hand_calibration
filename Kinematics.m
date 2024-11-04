function [endEffectorToBaseTform] = Kinematics(dobot, qs)
numOfPoses = size(qs, 1);

endEffectorToBaseTform = rigidtform3d.empty(numOfPoses, 0);

    for i=1:1:numOfPoses
        disp("Getting End-effector transform for pose " + i);
        dobot.PublishTargetJoint(qs(i, :));
        pause(2)
        tr = dobot.getCurrentEndEffectorPose;
        rotationMatrix = tr(1:3, 1:3);
        translationVector = tr(1:3, 4)';

        % Create the rigidtform3d object
        tform = rigidtform3d(rotationMatrix, translationVector);

        endEffectorToBaseTform(i) = tform;
    end

    disp("Finished getting End-effector transforms")
end