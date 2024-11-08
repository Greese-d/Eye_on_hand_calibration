classdef MotomanHC10DTP < RobotBaseClass
    %% MotomanHC10DTP
    % This class is based on the MotomanHC10DTP. 
    % URL: https://www.yaskawa.eu.com/products/robots/collaborative/productdetail/product/hc10dtp_17024
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'MotomanHC10DTP';
    end

    methods (Access = public) 
%% Constructor 
        function self = MotomanHC10DTP(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end
            
            self.PlotAndColourRobot();         
        end		

%% CreateModel
        function CreateModel(self)       
            link(1) = Link('d',0.275,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',0,'a',0.7,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
            link(3) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',0.5,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0.162,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(6) = Link('d',0.170,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            
            self.model = SerialLink(link,'name',self.name); 
        end    
    end
end