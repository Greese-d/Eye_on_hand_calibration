classdef Fetch < RobotBaseClass
    % This class is based on the Fetch. 
    % URL: https://docs.fetchrobotics.com/robot_hardware.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'Fetch';
    end   


    methods (Access = public) 
%% Constructor 
        function self = Fetch(baseTr)
			self.CreateModel();
            
			% Rotate robot to the correct orientation
            self.model.base = self.model.base.T * transl([0 0.0254 0.734]);
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end 

            self.model.tool = trotx(pi);		
			
            self.PlotAndColourRobot();         
        end
    
%% CreateModel
        function CreateModel(self)         
            link(1) = Link('d',0.05,   'a',0.117,  'alpha',-pi/2,   'qlim',deg2rad([-92 92]),   'offset', 0);   %shoulder pan
            link(2) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-70 87]),   'offset',pi/2); %shoulder lift
            link(3) = Link('d',0.35,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([-360 360]), 'offset', 0);   %uperarm roll
            link(4) = Link('d',0,      'a',0,      'alpha',pi/2,   'qlim',deg2rad([-129 129]),'offset', 0);    %elbow flex
            link(5) = Link('d',0.32,   'a',0,      'alpha',-pi/2,   'qlim',deg2rad([-360,360]), 'offset',0);    %forearm roll
            link(6) = Link('d',0,      'a',0,      'alpha',pi/2,  'qlim',deg2rad([-125,125]), 'offset', 0);   %wrist flex
            link(7) = Link('d',0.15,   'a',0,      'alpha',0,      'qlim',deg2rad([-360,360]), 'offset', 0);   %wrist roll

            self.model = SerialLink(link,'name',self.name);
        end       
    end
end