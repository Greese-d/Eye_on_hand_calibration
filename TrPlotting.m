
figure;
hold on
axis equal
axis([-1 1 -1 1 -1 1])

trplot(lastlocation, 'frame', 'endEffector')
trplot(lastlocation * cameraToEndEffectorTform.A, 'frame', 'camera')