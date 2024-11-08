%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)
    
    intersectionPoint = [0 0 0];
    u = point2OnLine - point1OnLine;
    w = point1OnLine - pointOnPlane;
    D = dot(planeNormal,u);
    N = -dot(planeNormal,w);
    check = 0; %#ok<NASGU>
    if abs(D) < 10^-7        % The segment is parallel to plane
        if N == 0           % The segment lies in plane
            check = 2;
            return
        else
            check = 0;       %no intersection
            return
        end
    end
    
    %compute the intersection parameter
    sI = N / D;
    intersectionPoint = point1OnLine + sI.*u;
    
    if (sI < 0 || sI > 1)
        check= 3;          %The intersection point  lies outside the segment, so there is no intersection
    else
        check=1;
    end
end           
