%% DrawPossibleEllipseGivenFoci
% For a set of two given foci (x,y) points of an ellipse, there are infinite
% combinations of what it would look like. However, if the eccentricity is given
% then there is enough information to draw it. For a fast handy drawing
% option it is useful to be able to have just the two foci and an assumed
% eccentricity (major is double the minor) to draw a non-axis aligned
% elipsoid with only foci as defined. Hence this function was created. 
%
% Note: To change the colours you could edit this line of code:
% plot_ellipse(E,[centerX,centerY])

%% Example test code
% close all 
% clear all
% while 1
%     f1 = rand(2,1)
%     f2 = rand(2,1)
%     plot(f1(1),f1(2),'r*')
%     hold on;
%     plot(f2(1),f2(2),'g*')
% 
%     DrawPossibleEllipseGivenFoci(f1,f2)
%     axis square
%     drawnow
%     pause(0.5)
% end

function DrawPossibleEllipseGivenFoci(focusPoint1, focusPoint2, eccentricity)
    if nargin < 3
        eccentricity = 2;
    end

    % Calculate center point of the ellipse
    centerX = (focusPoint1(1) + focusPoint2(1)) / 2;
    centerY = (focusPoint1(2) + focusPoint2(2)) / 2;

    % Rotation angle    
    theta = atan2(focusPoint2(2) - focusPoint1(2), focusPoint2(1) - focusPoint1(1));

    % Calculate distance between the foci
    distance = norm(focusPoint1 - focusPoint2);

    % Calculate length of minor axis
    minorAxis = distance / (2 * eccentricity);

    % Calculate length of major axis
    majorAxis = 2 * minorAxis;
    
    % Calculate the eigenvalues and eigenvectors
    lambdaMajor = (2*majorAxis)^2;
    lambdaMinor = (2*minorAxis)^2;
    vMajor = [cos(theta); sin(theta)];
    vMinor = [-sin(theta); cos(theta)];
    E = vMajor * lambdaMajor * vMajor' + vMinor * lambdaMinor * vMinor';    

    plot_ellipse(E,[centerX,centerY])

    axis equal;
end