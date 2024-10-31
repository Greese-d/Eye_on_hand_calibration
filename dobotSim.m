figure;
axis([-2 2 -2 2 0 2])
r = DobotMagician();

r.model.teach(zeros(1, 5))

q1 = [-0.1363   -0.1002   -0.0312         0];

%r.model.animate(q1)
