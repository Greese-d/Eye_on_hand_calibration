function extrinsicsArray = getExtrinsics(folder_calibPics, intrinsics, worldPoints)
    % Initialize output array
    extrinsicsArray = [];

    % Path to your images
    imageFolder = fullfile(folder_calibPics);
    images = imageDatastore(imageFolder);

    % Loop through each image in the folder
    for i = 1:numel(images.Files)
        % Load and display the current image
        imOrig = readimage(images, i);
        figure;
        imshow(imOrig);
        title("Input Image");

        % Undistort the image
        [im, newIntrinsics] = undistortImage(imOrig, intrinsics, OutputView="full");

        % Detect checkerboard points
        [imagePoints, boardSize] = detectCheckerboardPoints(im);

        % Check for checkerboard detection and validate points are finite
        if isempty(imagePoints) || any(~isfinite(imagePoints(:)))
            warning('No valid checkerboard detected in image %d. Skipping...', i);
            continue;
        end

        % Adjust for coordinate system shift
        newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
        imagePoints = imagePoints + newOrigin;

        % Estimate extrinsics
        camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);
        extrinsicsArray = [extrinsicsArray; camExtrinsics]; % Store the result

        % Calculate the camera pose
        camPose = extr2pose(camExtrinsics);

        % Plot the camera and reference object points
        figure;
        plotCamera(AbsolutePose=camPose, Size=20);
        hold on;
        pcshow([worldPoints, zeros(size(worldPoints, 1), 1)], ...
            VerticalAxisDir="down", MarkerSize=40);
    end

    % Return or save extrinsics for all images in the array
end