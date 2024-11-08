classdef UR10 < RobotBaseClass
    %% UR10 
    % Universal Robot 10kg payload robot model
    % URL: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR10';        
    end
    
    methods
%% Constructor
        function self = UR10(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);                
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.128,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.5716,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.16389,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.09037,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end          
    end
end