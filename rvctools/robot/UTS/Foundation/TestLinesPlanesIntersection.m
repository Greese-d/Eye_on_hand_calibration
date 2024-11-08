function TestLinesPlanesIntersection()
    close all

    planeNormals = [1,0,0; 0,1,0; -1,0,0; 0,-1,0];
    pointsOnPlane = [-1,-1,0; -1,-1,0; 1,1,0; 1,1,0];
    
    pointCount = 1000;
    startPoint = [0,0,0];
    point1OnLineSet = repmat(startPoint,pointCount,1);
    point2OnLineSet = 10*(rand(pointCount,3)-0.5);
    
    hold on;
    plot3(point1OnLineSet(:,1),point1OnLineSet(:,2),point1OnLineSet(:,3),'r*')
    plot3(point2OnLineSet(:,1),point2OnLineSet(:,2),point2OnLineSet(:,3),'b*')
    
    % We can visualise this as follows by first creating and
    % plotting a plane, which conforms to the previously defined planePoint and planeNormal                
    [Y,Z] = meshgrid(-1:0.1:1,-2:0.1:2);
    X = repmat(1,size(Y,1),size(Y,2));
    surf(X,Y,Z);
    
    [X,Z] = meshgrid(-1:0.1:1,-2:0.1:2);
    Y = repmat(1,size(Y,1),size(Y,2));
    surf(X,Y,Z);
    
    [Y,Z] = meshgrid(-1:0.1:1,-2:0.1:2);
    X = repmat(-1,size(Y,1),size(Y,2));
    surf(X,Y,Z);
    
    [X,Z] = meshgrid(-1:0.1:1,-2:0.1:2);
    Y = repmat(-1,size(Y,1),size(Y,2));
    surf(X,Y,Z);
    
    profile clear; profile on;
    [intersectionPoints] = LinesPlanesIntersection(planeNormals,pointsOnPlane,point1OnLineSet,point2OnLineSet);
    profile off; profile viewer;

    plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');

end           
