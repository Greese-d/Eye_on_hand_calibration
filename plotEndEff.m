figure
axis equal
axis (0.1*[-0.5 3 -1 1 0 2])
hold on;
plot(0, 0, "o")
view(50, 20)
for i=1:1:size(calibration.endEffectorToBaseTform, 2)
    transform = endEffectorToBaseTform(i)
    trplot(endEffectorToBaseTform(i).A)
    %plotCamera(AbsolutePose=transform, Size=0.005);
end
