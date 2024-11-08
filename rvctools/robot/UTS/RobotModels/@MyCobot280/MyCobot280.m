classdef MyCobot280 < RobotBaseClass
    %% MyCobot280 - "Baby-Elephant Collaborative Robotic Arm"
    % URL: https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.1-280/2.1.4.1%20Introduction%20of%20product%20parameters.html
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)  
        plyFileNameStem = 'MyCobot280';       
    end
    methods (Access = public)
%% Constructor 
        function self = MyCobot280(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end            

            % Overiding the default workspace for this small robot
            self.workspace = [-0.8 0.8 -0.8 0.8 -0.01 0.5];   

            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d', 0.13156, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-165 165]));
            link(2) = Link('d', -0.06639, 'a', 0.1104, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(3) = Link('d', 0.06639, 'a', 0.096, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            link(4) = Link('d', -0.06639, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-165 165]));
            link(5) = Link('d', 0.07318, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-165 165]));
            link(6) = Link('d', -0.0436, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-175 175]));

            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end