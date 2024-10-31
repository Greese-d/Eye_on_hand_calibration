% List available cameras
camList = webcamlist;
disp('Available cameras: ');
disp(camList);

% Initialize the webcam (adjust the index if necessary based on available cameras)
cam = webcam(2);  % Ensure that index 2 is valid; change if necessary

% Capture an image
rgbImage = snapshot(cam);

% Display the RGB image
figure;
imshow(rgbImage);
title('Captured RGB image');

% Convert RGB image to HSV
hsvImage = rgb2hsv(rgbImage);

% Display the HSV image
figure;
imshow(hsvImage);
title('Captured HSV image');
