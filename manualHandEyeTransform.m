function cameraToEndEffectorTform = manualHandEyeTransform(camExtrinsics, endEffectorToBaseTform, config)
    % Function to estimate the camera-to-end-effector transformation based on given camera
    % and end-effector transformations using the Tsai-Lenz method.
    %
    % Inputs:
    %   camExtrinsics - Array of camera extrinsic transformations (4x4xN matrix).
    %   endEffectorToBaseTform - Array of end-effector-to-base transformations (4x4xN matrix).
    %   config - Configuration, either "eye-in-hand" or "eye-to-hand".
    %
    % Output:
    %   cameraToEndEffectorTform - Estimated transformation from camera to end-effector.

    % Validate input configuration
    if ~strcmp(config, "eye-in-hand") && ~strcmp(config, "eye-to-hand")
        error("Unsupported configuration. Use 'eye-in-hand' or 'eye-to-hand'.");
    end

    % Ensure inputs are 4x4xN
    numPoses = size(camExtrinsics, 3);
    assert(numPoses == size(endEffectorToBaseTform, 3), 'Mismatch in number of poses');

    % Prepare matrices for the Tsai-Lenz equations
    A = [];
    B = [];

    for i = 1:numPoses - 1
        % Get relative transformations between consecutive poses
        if strcmp(config, "eye-in-hand")
            deltaA = inv(endEffectorToBaseTform(:,:,i)) * endEffectorToBaseTform(:,:,i+1);
            deltaB = camExtrinsics(:,:,i) * inv(camExtrinsics(:,:,i+1));
        else % "eye-to-hand"
            deltaA = inv(camExtrinsics(:,:,i)) * camExtrinsics(:,:,i+1);
            deltaB = endEffectorToBaseTform(:,:,i) * inv(endEffectorToBaseTform(:,:,i+1));
        end

        % Extract rotation components
        RA = deltaA(1:3, 1:3);
        RB = deltaB(1:3, 1:3);

        % Construct matrix equations for rotation (Ax = b)
        thetaA = vrrotmat2vec(RA);
        thetaB = vrrotmat2vec(RB);

        A = [A; skew(thetaA(1:3))];
        B = [B; thetaB(1:3)];
    end

    % Solve for rotation (Least squares solution for Ax = b)
    thetaX = A \ B;
    RX = vrrotvec2mat(thetaX);

    % Solve for translation
    tX = zeros(3,1);
    for i = 1:numPoses
        tA = endEffectorToBaseTform(1:3, 4, i);
        tB = camExtrinsics(1:3, 4, i);
        tX = tX + RX * tA - tB;
    end
    tX = tX / numPoses;

    % Output the result as a rigidtform3d object
    cameraToEndEffectorTform = rigidtform3d([RX, tX; 0, 0, 0, 1]);
end

% Helper function to compute the skew-symmetric matrix for vector
function S = skew(v)
    S = [  0, -v(3),  v(2);
         v(3),    0, -v(1);
        -v(2),  v(1),    0];
end

% Helper function to convert a rotation matrix to a rotation vector (axis-angle)
function thetaVec = vrrotmat2vec(R)
    angle = acos((trace(R) - 1) / 2);
    if angle == 0
        thetaVec = [0; 0; 0];
    else
        vec = (1 / (2 * sin(angle))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
        thetaVec = vec * angle;
    end
end

% Helper function to convert a rotation vector (axis-angle) to a rotation matrix
function R = vrrotvec2mat(thetaVec)
    angle = norm(thetaVec);
    if angle == 0
        R = eye(3);
    else
        axis = thetaVec / angle;
        S = skew(axis);
        R = eye(3) + sin(angle) * S + (1 - cos(angle)) * S^2;
    end
end