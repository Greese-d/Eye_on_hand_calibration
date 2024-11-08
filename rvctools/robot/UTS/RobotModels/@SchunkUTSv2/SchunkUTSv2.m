classdef SchunkUTSv2 < RobotBaseClass
    %% SchunkUTSv2
    % This class is based on the SchunkUTSv2 which is a custom-made arm for
    % the UTS grit blasting project 
    % 
    % WARNING: This model has been created by UTS students. No guarentee is
    % made about the accuracy or correctness of the of the DH parameters of
    % the accompanying ply files. Do not assume that this matches the real
    % robot! 

    properties(Access = public)   
        plyFileNameStem = 'SchunkUTSv2'; % Options: (-) SchunkUTSv2, (-) SchunkUTSv2EndEffectorNozzleNoSensor, (-) SchunkUTSv2EndEffectorPlate'
    end

    methods (Access = public) 
%% Constructor
        function self = SchunkUTSv2(baseTr)
			self.CreateModel();
			self.homeQ = [0,0,pi/2,0,0,0];
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end
            self.model.base = self.model.base.T * trotz(-pi/2) * trotx(pi);

            self.PlotAndColourRobotMat();
        end

%% CreateModel
        function CreateModel(self)
            link1 = Link('alpha',-pi/2,'a',0,'d',-0.38,'offset',0,'qlim',[deg2rad(-117),deg2rad(117)]);
            link2 = Link('alpha',pi,'a',0.385,'d',0,'offset',pi/2,'qlim',[deg2rad(-115),deg2rad(115)]);
            link3 = Link('alpha',pi/2,'a',0,'d',0,'offset',-pi/2,'qlim',[deg2rad(-110),deg2rad(110)]);
            link4 = Link('alpha',pi/2,'a',0,'d',-0.445,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)]);
            link5 = Link('alpha',-pi/2,'a',0,'d',0,'offset',0,'qlim',[deg2rad(-107),deg2rad(107)]);
            link6 = Link('alpha',pi,'a',0,'d',-0.2106,'offset',0,'qlim',[deg2rad(-200),deg2rad(200)]);
            
            self.model = SerialLink([link1 link2 link3 link4 link5 link6],'name',self.name);
        end  
    end
end