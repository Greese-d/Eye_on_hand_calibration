classdef IRB120 < RobotBaseClass
    %% IRB120 - from ABB 
    % URL: https://new.abb.com/products/robotics/robots/articulated-robots/irb-120
    %
    % ABB’s 6 axis robot – for flexible and compact production. It is ideal
    % for material handling and assembly applications and provides an
    % agile, compact and lightweight solution with superior control and
    % path accuracy. 
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)  
        plyFileNameStem = 'IRB120';       
    end
    methods (Access = public)
%% Constructor 
        function self = IRB120(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end            

            % Overiding the default workspace for this small robot
            self.workspace = [-0.6 0.6 -0.6 0.6 -0.01 0.8];   

            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([0      0.29       0         -pi/2  0]);
            link(2) = Link([-pi/2  0          0.27      0      0]);
            link(3) = Link([0      0          0.07      -pi/2  0]);
            link(4) = Link([0      0.302      0         pi/2   0]);
            link(5) = Link([0      0          0         -pi/2  0]);
            link(6) = Link([0      0.072      0          0     0]);

            % Incorporate joint limits
            link(1).qlim = [-165 165]*pi/180;
            link(2).qlim = [-110 110]*pi/180;
            link(3).qlim = [-110 70]*pi/180;
            link(4).qlim = [-160 160]*pi/180;
            link(5).qlim = [-120 120]*pi/180;
            link(6).qlim = [-400 400]*pi/180;

            % link(1).offset =  -pi;
            link(2).offset =  -pi/2;
            % link(4).offset =  pi;
            % link(5).offset =  pi;

            % link(1) = Link('d', 0.13156, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-165 165]));
            % link(2) = Link('d', -0.06639, 'a', 0.1104, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            % link(3) = Link('d', 0.06639, 'a', 0.096, 'alpha', 0, 'qlim', deg2rad([-165 165]));
            % link(4) = Link('d', -0.06639, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-165 165]));
            % link(5) = Link('d', 0.07318, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-165 165]));
            % link(6) = Link('d', -0.0436, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-175 175]));

            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end