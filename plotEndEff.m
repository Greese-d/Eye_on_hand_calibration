figure
axis equal
axis (0.1*[-0.5 3 -1 1 -1 2])
hold on;
plot(0, 0, "o")
view(50, 20)
for i=1:1:size(calibration.endEffectorToBaseTform, 2)
    trplot(eye(4),'color', 'm')
    transform = calibration.endEffectorToBaseTform(i);
    trplot(calibration.endEffectorToBaseTform(i).A)
    %calibration.boardToCameraTform(i)
    %trplot(inv(calibration.boardToCameraTform(i).A))
    %trplot(calibration.boardToCameraTform(i).A)
    trplot(calibration.endEffectorToBaseTform(i).A * calibration.cameraToEndEffectorTform.A, 'color', 'g')
    %plotCamera(AbsolutePose=transform, Size=0.005);
end
