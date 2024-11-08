function cameraToEndEffectorTform = helperEstimateHandEyeTransform4DOF(boardToCameraTform, endEffectorToBaseTform)
    numPoses = numel(boardToCameraTform);

    PEndEffectorIToJ = repmat(zeros(3,1), 1, 1, numPoses-1);
    PCameraIToJ = PEndEffectorIToJ;

    for i = 1:(numPoses - 1)
        j = i + 1;

        % Collect the 4 transforms of interest.
        TCameraI = boardToCameraTform(i).A;
        TCameraJ = boardToCameraTform(j).A; 
        TEndEffectorI = endEffectorToBaseTform(i).A;
        TEndEffectorJ = endEffectorToBaseTform(j).A;

        % Compute relative transformations and rotation axes.
        TEndEffectorIJ = TEndEffectorJ \ TEndEffectorI;
        TCameraIJ = (TCameraI' \ TCameraJ')';

        axangCIJ = tform2axang(TCameraIJ);
        axangGIJ = tform2axang(TEndEffectorIJ);
        thetaCIJ = axangCIJ(4);
        thetaGIJ = axangGIJ(4);
        PCameraIToJ(:,:,i) = 2 * sin(thetaCIJ / 2) * axangCIJ(1:3)';
        PEndEffectorIToJ(:,:,i) = 2 * sin(thetaGIJ / 2) * axangGIJ(1:3)';
    end

    % Least squares for rotation estimation
    A = zeros((numPoses - 1) * 3, 3);
    b = zeros((numPoses - 1) * 3, 1);
    for i = 1:(numPoses - 1)
        A((i-1) * 3 + 1:i * 3, :) = helperSkewMatrix(PEndEffectorIToJ(:,:,i) + PCameraIToJ(:,:,i));
        b((i-1) * 3 + 1:i * 3, 1) = PCameraIToJ(:,:,i) - PEndEffectorIToJ(:,:,i);
    end

    % Solve for the rotation
    [PEndEffectorToCameraUnscaled, ~] = lsqr(A, b);
    PEndEffectorToCameraScaled = (2 * PEndEffectorToCameraUnscaled) / sqrt(1 + norm(PEndEffectorToCameraUnscaled)^2);

    % Tsai, Lenz Equation for rotation
    PSkew = helperSkewMatrix(PEndEffectorToCameraScaled);
    REndEffectorToCamera = (1 - 0.5 * norm(PEndEffectorToCameraScaled)^2) * eye(3) + 0.5 * (PEndEffectorToCameraScaled * ...
        PEndEffectorToCameraScaled' + sqrt(4 - norm(PEndEffectorToCameraScaled)^2) * PSkew);

    % Clear A and b
    A(:,:) = 0;
    b(:,:) = 0;

    % Estimate translation part
    for i = 1:(numPoses - 1)
        j = i + 1;
        TCameraI = boardToCameraTform(i).A;
        TCameraJ = boardToCameraTform(j).A;
        TEndEffectorI = endEffectorToBaseTform(i).A;
        TEndEffectorJ = endEffectorToBaseTform(j).A;
        TEndEffectorIJ = TEndEffectorJ \ TEndEffectorI;
        TCameraIJ = (TCameraI' \ TCameraJ')';

        A((i - 1) * 3 + 1:i * 3, :) = TEndEffectorIJ(1:3, 1:3) - eye(3);
        b((i - 1) * 3 + 1:i * 3, 1) = REndEffectorToCamera * TCameraIJ(1:3, 4) - TEndEffectorIJ(1:3, 4);
    end

    % Solve for translation
    [TranslationEndEffectorToCamera, ~] = lsqr(A, b);

    TEndEffectorToCamera = trvec2tform(TranslationEndEffectorToCamera') * rotm2tform(REndEffectorToCamera);
    cameraToEndEffectorTform = rigidtform3d(TEndEffectorToCamera);
end