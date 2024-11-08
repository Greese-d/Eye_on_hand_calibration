%% LinesTriangleIntersection
% Given a set of plane (normals of planes and points on these planes) and a
% set of lines defined as start and end points get the intersection points
% between these lines (within the line segment) and the infinite planes
function [allIntersectionPoints] = LinesTriangleIntersection(vertices,faces,point1OnLineSet,point2OnLineSet)
    assert(size(point1OnLineSet,2) == size(point2OnLineSet,2));
    pointCount = size(point1OnLineSet,1);
    lineDirections = point2OnLineSet - point1OnLineSet;

    v1 = vertices(faces(:,1),:);
    v2 = vertices(faces(:,2),:);
    v3 = vertices(faces(:,3),:);
    
    % Compute normal vector of the triangle
    planeNormals = cross(v2 - v1, v3 - v1);%     
    
    allIntersectionPoints = [];
    intersectionPoints = [];

    for i = 1:size(planeNormals,1)
        possibleLinesThatIntersect = true(pointCount,1);
        
        pointOnPlanev1 = v1(i,:);        
        pointOnPlanev1RepMat = repmat(pointOnPlanev1,pointCount,1);   
        
        % Check if line intersects plane of the triangle
        possibleLinesThatIntersect(dot(pointOnPlanev1RepMat, lineDirections,2) == 0) = false; % line is parallel to plane so move there will be no intersections
        
        planeNormal = planeNormals(i,:);
        planeNormalRepMat = repmat(planeNormal,pointCount,1);  

        t = dot(planeNormalRepMat, pointOnPlanev1RepMat - point1OnLineSet,2) ./ dot(planeNormalRepMat, lineDirections,2); % compute t-value of intersection point

        possibleLinesThatIntersect(t < 0 | t > 1) = false; % intersection point is outside of the line segment

        intersectionPoints = point1OnLineSet + t .* lineDirections; % compute intersection point

        % h1 = plot3(intersectionPoints(possibleLinesThatIntersect,1)...
        %      ,intersectionPoints(possibleLinesThatIntersect,2)...
        %      ,intersectionPoints(possibleLinesThatIntersect,3),'g*');
        % axis([-2,2,-2,2,-2,2])

        pointOnPlanev2 = v2(i,:);
        pointOnPlanev2RepMat = repmat(pointOnPlanev2,pointCount,1);
        pointOnPlanev3 = v3(i,:); 
        pointOnPlanev3RepMat = repmat(pointOnPlanev3,pointCount,1);

        % Check if intersection point is inside triangle
        u = pointOnPlanev2RepMat - pointOnPlanev1RepMat;
        w = intersectionPoints - pointOnPlanev1;
        uu = dot(u, u,2);
        uv = dot(u, pointOnPlanev3RepMat - pointOnPlanev1,2);
        uw = dot(u, w,2);
        vv = dot(pointOnPlanev3RepMat - pointOnPlanev1, pointOnPlanev3RepMat - pointOnPlanev1,2);
        vw = dot(pointOnPlanev3RepMat - pointOnPlanev1, w,2);
        det = uv .* uv - uu .* vv;
        s = (uv .* vw - vv .* uw) ./ det;
        t = (uv .* uw - uu .* vw) ./ det;
        possibleLinesThatIntersect(s < 0 | t < 0 | 1 < s + t) = false;
        % h2 = plot3(intersectionPoints(possibleLinesThatIntersect,1)...
        %      ,intersectionPoints(possibleLinesThatIntersect,2)...
        %      ,intersectionPoints(possibleLinesThatIntersect,3),'k*');
        % axis([-2,2,-2,2,-2,2])

        allIntersectionPoints = [allIntersectionPoints;intersectionPoints(possibleLinesThatIntersect,:)];
       
            % delete(h1);delete(h2);        
    end
end        
