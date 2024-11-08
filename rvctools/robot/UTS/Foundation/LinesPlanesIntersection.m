%% LinesPlanesIntersection
% Given a set of plane (normals of planes and points on these planes) and a
% set of lines defined as start and end points get the intersection points
% between these lines (within the line segment) and the infinite planes
function [intersectionPoints] = LinesPlanesIntersection(planeNormals,pointsOnPlane,point1OnLineSet,point2OnLineSet)
    
    assert(size(planeNormals,2) == size(pointsOnPlane,2));
    assert(size(point1OnLineSet,2) == size(point2OnLineSet,2));
    pointCount = size(point1OnLineSet,1);

    intersectionPoints = [];

    for i = 1:size(planeNormals,1)
        pointOnPlane = pointsOnPlane(i,:);        
        pointOnPlaneRepMat = repmat(pointOnPlane,pointCount,1);

        planeNormal = planeNormals(i,:);
        planeNormalRepMat = repmat(planeNormal,pointCount,1);              

        u = point2OnLineSet - point1OnLineSet;
        w = point1OnLineSet - pointOnPlaneRepMat;
        D = dot(planeNormalRepMat,u,2);
        N = -dot(planeNormalRepMat,w,2);       
        
        %compute the intersection parameter
        sI = N ./ D;       

        validIndex = 0 < sI ...
                  & sI < 1 ...
                  & 10^-7 < abs(D);

        intersectionPoints = [intersectionPoints; point1OnLineSet(validIndex,:) + sI(validIndex,:).*u(validIndex,:)];        %#ok<AGROW>
    end
end        
