function camExtrinsicsArray = Extrinsics2(folderName)
    % Load the images
    images = imageDatastore(folderName);

    % Detect the checkerboard corners in the images
    [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

    % Generate the world coordinates of the checkerboard corners
    squareSize = 29; % size in millimeters
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Calibrate the camera using the first image
    I = readimage(images, 1); 
    imageSize = [size(I, 1), size(I, 2)];
    cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
        ImageSize=imageSize);
    intrinsics = cameraParams.Intrinsics;

    % Initialize a struct array to hold rotation and translation for each image
    numImages = length(images.Files);
    camExtrinsicsArray(numImages) = struct('Rotation', [], 'Translation', []); % Struct array

    % Loop through each image to calculate extrinsics
    for i = 1:numImages
        imOrig = readimage(images, i);
        
        % Undistort the image
        [im, newIntrinsics] = undistortImage(imOrig, intrinsics, OutputView="full");
        
        % Find the reference object in the new image
        [imagePoints, ~] = detectCheckerboardPoints(im);
        
        % Compensate for the image coordinate system shift
        newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
        imagePoints = imagePoints + newOrigin;
        
        % Calculate new extrinsics
        camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);
        
        % Extract rotation and translation and store them
        camPose = extr2pose(camExtrinsics);
        camExtrinsicsArray(i).Rotation = camPose.Rotation;
        camExtrinsicsArray(i).Translation = camPose.Translation;
    end
end