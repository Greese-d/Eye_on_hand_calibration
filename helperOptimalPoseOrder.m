function orderOfPoses = helperOptimalPoseOrder(TCameraToBoard)
    % Create necessary vectors.
    numPoses = size(TCameraToBoard,3);
    anglesBetween = ones(numPoses-1,1);
    orderOfPoses = 1:numPoses;
    
    % Iterate over the indices to choose each subsequent pose.
    for i = 1:numPoses-2
        TCameraI = TCameraToBoard(:,:,i);

        % Collect the angles between pose i and the remaining poses.
        for j = i+1:numPoses
            TCameraJ = TCameraToBoard(:,:,j);
            TCameraIJ = TCameraI \ TCameraJ;
            axangcij = tform2axang(TCameraIJ);
            anglesBetween(j) = axangcij(4);
        end

        % Select the pose with the maximum angle to appear next in the
        % ordering.
        [~, idMax] = max(anglesBetween(i:end));
        tmp = orderOfPoses(i+1);
        orderOfPoses(i+1) = orderOfPoses(idMax+i-1);
        orderOfPoses(idMax+i-1) = tmp;
    end
end