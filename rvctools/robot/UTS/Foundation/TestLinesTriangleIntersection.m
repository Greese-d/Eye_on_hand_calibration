function TestLinesTriangleIntersection()
    close all

    % Define set of triangles by vertices and faces
    vertices = [0 0 0; 0 1 0; 1 1 0; 1 0 0; 0 0 1; 0 1 1; 1 1 1; 1 0 1]; % 8 vertices of an open cube
    % Shift cube  
    vertices(:,1) = vertices(:,1) - 0.1;
    faces = [1 5 6; 1 6 2; 2 6 7; 2 7 3; 3 7 8; 3 8 4; 4 8 5; 4 5 1]; % 12 triangles of the cube

    % Plot the triangles
    figure;
    patch('Vertices',vertices,'Faces',faces,'FaceColor','red','EdgeColor','black');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis equal; 
    
    % Create some start and end points
    pointCount = 20;
    startPoint = [0.5,0.5,0.5];
    point1OnLineSet = repmat(startPoint,pointCount,1);
    point2OnLineSet = 2*(rand(pointCount,3)-0.5);
    
    % Plot the start and end points
    hold on;
    plot3(point1OnLineSet(:,1),point1OnLineSet(:,2),point1OnLineSet(:,3),'r*')
    plot3(point2OnLineSet(:,1),point2OnLineSet(:,2),point2OnLineSet(:,3),'b*')      
    
    % profile clear; profile on;
    [intersectionPoints] = LinesTriangleIntersection(vertices,faces,point1OnLineSet,point2OnLineSet);
    % profile off; profile viewer;

    % Plot the points of intersection
    plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');
end           