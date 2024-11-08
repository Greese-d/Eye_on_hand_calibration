classdef UR10e < RobotBaseClass
    %% UR10e
    % Universal Robot 10kg payload robot model - series 'e'
    % URL: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR10e';        
    end
    
    methods
%% Constructor
        function self = UR10e(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end          
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.1807,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(3) = Link('d',0,'a',-0.5716,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);     
            link(4) = Link('d',0.17415,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.11985,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            link(6) = Link('d',0.11655,'a',0,'alpha',0,'qlim',deg2rad([-360 360]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end          
    end
end