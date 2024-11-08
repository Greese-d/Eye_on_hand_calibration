function cameraToEndEffectorTform = helperEstimateHandEyeTransform(boardToCameraTform, endEffectorToBaseTform, configuration)
    arguments
        boardToCameraTform (:,1) rigidtform3d
        endEffectorToBaseTform (:,1) rigidtform3d
        configuration {mustBeMember(configuration, ["eye-in-hand","eye-to-hand"])}
    end
    numPoses = size(boardToCameraTform,1);

    % In the eye-to-hand case, the camera is mounted in the environment and
    % the calibration board is mounted to the robot end-effector joint.
    if configuration == "eye-to-hand"
        for i = 1:numPoses
            currAInv = boardToCameraTform(i).invert().A;
            endEffectorToBaseTform(i) = rigidtform3d(currAInv);
        end
    end
    
    % Reorder the poses to have greater angles between each pair.
    orderOfPoses = helperOptimalPoseOrder(boardToCameraTform);
    boardToCameraTform(:,:,:) = boardToCameraTform(:,:,orderOfPoses);
    endEffectorToBaseTform(:,:,:) = endEffectorToBaseTform(:,:,orderOfPoses);

    PEndEffectorIToJ=repmat(zeros(3,1), 1, 1, numPoses-1);
    PCameraIToJ=PEndEffectorIToJ;

    % Iterate through pairs of poses to determine transformation angle.
    for i = 1:numPoses-1
        j=i+1;

        % Collect the 4 transforms of interest.
        TCameraI = boardToCameraTform(i).A;
        TCameraJ = boardToCameraTform(j).A; 
        TEndEffectorI = endEffectorToBaseTform(i).A;
        TEndEffectorJ = endEffectorToBaseTform(j).A;

        % Get transforms grom end-effector joint I to end-effector joint J, and for camera I and
        % camera J.
        TEndEffectorIJ = TEndEffectorJ\TEndEffectorI;
        TCameraIJ = (TCameraI'\TCameraJ')';

        % Get the axes and angles of the for both transforms.
        axangCIJ = tform2axang(TCameraIJ);
        axangGIJ = tform2axang(TEndEffectorIJ);
        thetaCIJ = axangCIJ(4);
        thetaGIJ = axangGIJ(4);
        PCameraIToJax = axangCIJ(1:3);
        PEndEffectorIToJax = axangGIJ(1:3);

        % P vectors contain the axis of rotation and are scaled to represent
        % the amount of rotation using Rodrigues' rotation formula.
        PCameraIToJ(:,:,i) = 2*sin(thetaCIJ/2)*PCameraIToJax';
        PEndEffectorIToJ(:,:,i) = 2*sin(thetaGIJ/2)*PEndEffectorIToJax';
    end

    % Set up least squares problem for rotation estimation.
    A=zeros((numPoses-1)*3,3);
    b=zeros((numPoses-1)*3,1);
    for i = 1:numPoses-1
        A((i-1)*3+1:i*3,:) = helperSkewMatrix(PEndEffectorIToJ(:,:,i) + PCameraIToJ(:,:,i));
        b((i-1)*3+1:i*3,1) = PCameraIToJ(:,:,i) - PEndEffectorIToJ(:,:,i);
    end
    [PEndEffectorToCameraUnscaled,~] = lsqr(A,b);
    PEndEffectorToCameraScaled = (2 * PEndEffectorToCameraUnscaled) / sqrt(1 + norm(PEndEffectorToCameraUnscaled)^2);

    % Find rotation given in Tsai, Lenz Equation 10.
    PSkew = helperSkewMatrix(PEndEffectorToCameraScaled);
    REndEffectorToCamera = (1 - (0.5 * norm(PEndEffectorToCameraScaled)^2)) * eye(3) + 0.5 * (PEndEffectorToCameraScaled * ...
        PEndEffectorToCameraScaled' + sqrt(4 - norm(PEndEffectorToCameraScaled)^2) * PSkew);
    
    % Clear A and b.
    A(:,:) = 0;
    b(:,:) = 0;

    % Iterate through numPoses-1 pairs of poses to find the translation part
    % of the transformation.
    for i = 1:numPoses-1
        j = i+1;

        % Use known transforms to compute transforms between poses.
        TCameraI = boardToCameraTform(i).A;
        TCameraJ = boardToCameraTform(j).A;
        TEndEffectorI = endEffectorToBaseTform(i).A;
        TEndEffectorJ = endEffectorToBaseTform(j).A;
        TEndEffectorIJ = TEndEffectorJ \ TEndEffectorI;
        TCameraIJ = (TCameraI' \ TCameraJ')';

        % Set up least squares to estimate translation.
        A((i-1)*3+1:i*3,:) = TEndEffectorIJ(1:3,1:3)-eye(3);
        b((i-1)*3+1:i*3,1) = REndEffectorToCamera*TCameraIJ(1:3,4)-TEndEffectorIJ(1:3,4);
    end

    % Compute translation using least squares.
    [TranslationEndEffectorToCamera,~] = lsqr(A,b);
    TEndEffectorToCamera = trvec2tform(TranslationEndEffectorToCamera')*rotm2tform(REndEffectorToCamera);
    cameraToEndEffectorTform = rigidtform3d(TEndEffectorToCamera);
end