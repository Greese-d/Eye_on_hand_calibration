classdef DensoVM6083 < RobotBaseClass
    %% DensoVM6083
    % This class is based on the DensoVM6083. 
    % URL: https://www.denso-wave.com/en/robot/product/five-six/vm.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'DensoVM6083';
    end

    methods (Access = public) 
        %% Constructor
        function self = DensoVM6083(baseTr)
			self.CreateModel();
			self.homeQ = [0,0,pi/2,0,0,0];
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end
            
            self.PlotAndColourRobotMat();
        end

%% Create the robot model
        function CreateModel(self)
            link1 = Link('d',0.475, 'a',0.180,  'alpha',-pi/2,  'offset',0,     'qlim',[deg2rad(-170), deg2rad(170)]);
            link2 = Link('d',0,     'a',0.385,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
            link3 = Link('d',0,     'a',-0.100, 'alpha',pi/2,   'offset',pi/2,  'qlim',[deg2rad(-80), deg2rad(165)]);
            link4 = Link('d',0.329+0.116,'a',0, 'alpha',-pi/2,  'offset',0,     'qlim',[deg2rad(-185), deg2rad(185)]);
            link5 = Link('d',0,     'a',0,      'alpha',pi/2,   'offset',0,     'qlim',[deg2rad(-120), deg2rad(120)]);
            link6 = Link('d',0.09,  'a',0,      'alpha',0,      'offset',0,     'qlim',[deg2rad(-360), deg2rad(360)]);

            self.model = SerialLink([link1 link2 link3 link4 link5 link6],'name',self.name);
        end    
    end
end

