%% Sawyer robot
classdef Sawyer < RobotBaseClass
    %% Sawyer 
    % RethinkRobotics's 7DOF 1,260 mm range 4kg payload robot
    % URL: https://www.rethinkrobotics.com/sawyer
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!
   
    properties(Access = public)   
        
        plyFileNameStem = 'Sawyer';        
    end
       
    methods (Access = public) 
        function self = Sawyer(baseTr)            
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end            
            self.homeQ = [0,-pi/2,0,-pi/2,0,pi/2,0];
            self.PlotAndColourRobot(); 
        end
                       
        %% CreateModel
        function CreateModel(self)
            % Create Sawyer Arm - Joint angles obtained from: http://mfg.rethinkrobotics.com/wiki/Robot_Hardware#tab=Sawyer
            link(1) = Link('d',0.317,'a',0.081,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 5.3213);
            link(2) = Link('d',0.1925,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-219),deg2rad(131)], ...
                        'm', 4.505);
            link(3) = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.745);
            link(4) = Link('d',-0.1685,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 2.5097);
            link(5) = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.1136);
            link(6) = Link('d',0.1363,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.5625);
            link(7) = Link('d',0.13375,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-270),deg2rad(270)], ...
                        'm', 0.3292);
            self.model = SerialLink(link,'name',self.name);
        end
                
% %% PlotAndColourRobot
% % Given a robot index, add the glyphs (vertices and faces) and
% % colour them in if data is available 
%         function PlotAndColourRobot(self)
%             mpath = strrep(which(mfilename),[mfilename '.m'],'');
% 
%             for linkIndex = 0:self.model.n
%                 [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([mpath,'SawyerLink',num2str(linkIndex),'.ply'],'tri');
% 
%                 self.model.faces{linkIndex + 1} = faceData;
% 
%                 % Obtain the faceData for the current link and save to the
%                 % cell.
%                 self.model.faces{linkIndex+1} = faceData;
% 
%                 % Obtain the vertexData for the current link and perform 
%                 % operations on the values depending on the link + save to 
%                 % the cell.
%                 if linkIndex == 0
%                     % Use the self.baseTr variable to shift the base (Link 0) of
%                     % the robot into the wanted location.
%                     % First obtain rotation and position, then multiple the vertex
%                     % position by the 3x3 rotation before adding the translation
%                     % (position).
%                     shift = false;
%                     if shift
%                         rot = self.baseTr(1:3, 1:3);
%                         pos = self.baseTr(1:3, 4);
%                         self.model.points{linkIndex+1} = (rot * vertexData(:, :)' + pos)';
%                     else
%                         self.model.points{linkIndex+1} = vertexData;
%                     end
% 
%                 else
%                     % If not Link 0 (> Link 0), no work has to be done to the
%                     % vertex values as the .base method moves every link from Link
%                     % 1 onwards.
%                     self.model.faces{linkIndex+1} = faceData;
%                     self.model.points{linkIndex+1} = vertexData;
%                 end
%             end
% 
%             % Display robot
%             self.model.plot3d(zeros(1,self.model.n),'noarrow','notiles');
%             if isempty(findobj(get(gca,'Children'),'Type','Light'))
%                 camlight
%             end  
%             self.model.delay = 0;
% 
%             % Try to correctly colour the arm (if colours are in ply file data)
%             for linkIndex = 0:self.model.n
%                 handles = findobj('Tag', self.model.name);
%                 h = get(handles,'UserData');
%                 try 
%                     h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
%                                                                   , plyData{linkIndex+1}.vertex.green ...
%                                                                   , plyData{linkIndex+1}.vertex.blue]/255;
%                     h.link(linkIndex+1).Children.FaceColor = 'interp';
%                 catch ME_1
%                     disp(ME_1);
%                     continue;
%                 end
%             end
%         end      
    end
    % methods (Static)
    %     %% GetModel
    %     % Given a name (optional), create and return a Sawyer robot model
    %     function robot = GetModel(name)
    %         if nargin < 1
    %             % Create a unique name (ms timestamp after 1ms pause)
    %             pause(0.001);
    %             name = ['SawyerRobot',datestr(now,'yyyymmddTHHMMSSFFF')];
    %         end
    %         % Create Sawyer Arm - Joint angles obtained from: http://mfg.rethinkrobotics.com/wiki/Robot_Hardware#tab=Sawyer
    %         L1 = Link('d',0.317,'a',0.081,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
    %                     'm', 5.3213);
    %         L2 = Link('d',0.1925,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-219),deg2rad(131)], ...
    %                     'm', 4.505);
    %         L3 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
    %                     'm', 1.745);
    %         L4 = Link('d',-0.1685,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
    %                     'm', 2.5097);
    %         L5 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
    %                     'm', 1.1136);
    %         L6 = Link('d',0.1363,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
    %                     'm', 1.5625);
    %         L7 = Link('d',0.13375,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-270),deg2rad(270)], ...
    %                     'm', 0.3292);
    %         robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
    %     end
    % end    
end