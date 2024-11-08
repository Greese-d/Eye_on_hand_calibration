classdef KinovaGen3 < RobotBaseClass
    %% KinovaGen3
    % This class is based on the KinovaGen3. 
    % URL: https://www.kinovarobotics.com/product/gen3-robots
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'KinovaGen3';
    end

    methods (Access = public) 
%% Constructor 
        function self = KinovaGen3(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end
            
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)       
            link(1) = Link('d',0.2433,'a',0,   'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-154.1) ,deg2rad(154.1)]);
            link(2) = Link('d',0.03,  'a',0.28,'alpha',0,   'offset',pi/2,'qlim',[deg2rad(-150.1) ,deg2rad(150.1)]);
            link(3) = Link('d',-.02,  'a',0,   'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-150.1) ,deg2rad(150.1)]);
            link(4) = Link('d',-.245, 'a',0,   'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-148.98),deg2rad(148.98)]);
            link(5) = Link('d',0.057, 'a',0,   'alpha',pi/2,'offset',0,'qlim',[deg2rad(-144.97),deg2rad(145.0)]);
            link(6) = Link('d',0.2622,'a',0,   'alpha',0,   'offset',0,'qlim',[deg2rad(-148.98),deg2rad(148.98)]);
            
            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end
