classdef DensoVS060 < RobotBaseClass
    %% DensoVS060
    % This class is based on the DensoVS060. 
    % URL: https://www.denso-wave.com/en/robot/product/five-six/vp.html
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)  
        plyFileNameStem = 'DensoVS060';
    end
    methods (Access = public)
%% Constructor 
        function self = DensoVS060(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end
            
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)       
            link(1) = Link('alpha',pi/2,'a',0, 'd',0.148, 'offset',pi,'qlim',[deg2rad(-170), deg2rad(170)] );
            link(2) = Link('alpha',0,'a',0.305, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            link(3) = Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-125), deg2rad(155)]);
            link(4) = Link('alpha',pi/2,'a',0, 'd',0.3, 'offset',0,'qlim',[deg2rad(-270), deg2rad(270)]);
            link(5) = Link('alpha',-pi/2,'a',0, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            link(6) = Link('alpha',0,'a',0, 'd',0.06, 'offset',0,'qlim',[deg2rad(-360), deg2rad(360)]);

            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end