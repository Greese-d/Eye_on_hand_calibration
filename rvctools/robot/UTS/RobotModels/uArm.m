%% uArm
% For the uArm from CCfactory. 
% Several equations have been translated to Matlab by Gavin based upon the
% project:
% UArmForROS 
% designed by Joey Song ( joey@ufactory.cc / astainsong@gmail.com)
% https://github.com/uArm-Developer/UArmForROS
%
% Example UTS student project: https://www.youtube.com/watch?v=8LqEBdlr2Ss

classdef uArm < handle
    properties (Constant)
        %> UARM SPECIFICATIONS (what units?)
        MATH_TRANS  = 57.2958
        MATH_L1	= (10.645+0.6)
        MATH_L2	= 2.117
        MATH_L3	= 14.825
        MATH_L4	= 16.02
        MATH_L43 = uArm.MATH_L4 / uArm.MATH_L3
        
        % UARM OFFSETS
        TopOffset = -1.5
        BottomOffset = 1.5
    end

    methods
        function self = uArm()
        end
        
        %% fkine
        % Closed form fkine solution
        function [position] = fkine(self,q)
            % This seems to be all that is required to do fkine
            g_l5 = (self.MATH_L2 + self.MATH_L3 * cos(q() / self.MATH_TRANS) + self.MATH_L4*cos(q(3) / self.MATH_TRANS));
            position = [ -cos( abs(q(1)/ self.MATH_TRANS))*g_l5 ...
                       , -sin(abs(q(1) / self.MATH_TRANS))*g_l5 ...
                       , self.MATH_L1 + self.MATH_L3 * sin(abs(q(2) / self.MATH_TRANS)) - self.MATH_L4 * sin(abs(q(3) / self.MATH_TRANS))];            
        end
        
        %% ikine
        % Closed form ikine solution
        function [q] = ikine(self,position)
            x = position(1);
            y = position(2);
            z = position(3);
                       
            if (self.MATH_L1 + self.MATH_L3 + self.TopOffset) < z
                z = self.MATH_L1 + self.MATH_L3 + self.TopOffset;
            end
            
            if z < (self.MATH_L1 - self.MATH_L4 + self.BottomOffset)
                z = self.MATH_L1 - self.MATH_L4 + self.BottomOffset;
            end

            g_y_in = (-y-self.MATH_L2)/self.MATH_L3;
            g_z_in = (z - self.MATH_L1) / self.MATH_L3;
            g_right_all = (1 - g_y_in*g_y_in - g_z_in*g_z_in - self.MATH_L43*self.MATH_L43) / (2 * self.MATH_L43);
            g_sqrt_z_y = sqrt(g_z_in*g_z_in + g_y_in*g_y_in);

            if x == 0
                % Calculate value of theta 1
                g_theta_1 = 90;
                % Calculate value of theta 3
                if g_z_in == 0
                    g_phi = 90;
                else
                    g_phi = atan(-g_y_in / g_z_in) * self.MATH_TRANS;
                end
                if g_phi > 0
                    g_phi = g_phi - 180;
                    g_theta_3 = asin(g_right_all / g_sqrt_z_y)*self.MATH_TRANS - g_phi;

                    if g_theta_3 < 0
                        g_theta_3 = 0;
                    end
                    % Calculate value of theta 2
                    g_theta_2 = asin((z + self.MATH_L4*sin(g_theta_3 / self.MATH_TRANS) - self.MATH_L1) / self.MATH_L3)*self.MATH_TRANS;
                end
            else
                % Calculate value of theta 1
                g_theta_1 = atan(y / x)*self.MATH_TRANS;
                if (y/x) < 0
                    g_theta_1 = g_theta_1 + 180;
                end
                if y == 0
                    if x > 0
                        g_theta_1 = 180;
                    else
                        g_theta_1 = 0;
                    end
                end
                % Calculate value of theta 3
                g_x_in = (-x / cos(g_theta_1 / self.MATH_TRANS) - self.MATH_L2) / self.MATH_L3;
                if g_z_in == 0
                    g_phi = 90;
                else
                    g_phi = atan(-g_x_in / g_z_in)*self.MATH_TRANS;
                end

                if g_phi > 0
                    g_phi = g_phi - 180 ;
                end

                g_sqrt_z_x = sqrt(g_z_in^2 + g_x_in^2);

                g_right_all_2 = -1 * (g_z_in^2 + g_x_in^2 + self.MATH_L43^2 - 1) / (2 * self.MATH_L43);
                g_theta_3 = asin(g_right_all_2 / g_sqrt_z_x) * self.MATH_TRANS;
                g_theta_3 = g_theta_3 - g_phi;

                if g_theta_3 < 0
                    g_theta_3 = 0;
                end
                % Calculate value of theta 2
                g_theta_2 = asin(g_z_in + self.MATH_L43*sin(abs(g_theta_3 / self.MATH_TRANS)))*self.MATH_TRANS;
            end

            g_theta_1 = abs(g_theta_1);
            g_theta_2 = abs(g_theta_2);

            position = self.fkine([g_theta_1,g_theta_2, g_theta_3]);
            if (position(2)>y+0.1) || (position(2)<y-0.1)
                g_theta_2 = 180 - g_theta_2;
            end

            if isnan(g_theta_1) || isinf(g_theta_1)
                g_theta_1 = self.readAngle(1);
            end

            if isnan(g_theta_2) || isinf(g_theta_2)
                g_theta_2 = self.readAngle(2);
            end

            if isnan(g_theta_3) || isinf(g_theta_3) || g_theta_3<0 
                g_theta_3 = self.readAngle(3);
            end

            q = [ g_theta_1, g_theta_2, g_theta_3];
        end        
    end
end