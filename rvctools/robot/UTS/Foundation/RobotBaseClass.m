classdef RobotBaseClass < handle
    %% RobotBaseClass The base robot class from which other UTS robot models should inherit 

    %#ok<*TRYNC>    

    properties (Abstract) % An inheriing class must implement and assign these properties 
        %> The first few letters or the name of ply files in the same directory
        plyFileNameStem;
    end

    properties
        %> Robot model
        model;
        
        % Name of the robot (to be copied into model.name)
        name;

        %> workspace (this changes based on the DH parameters)  
        workspace = [-10 10 -10 10 -0.01 10];

        %> The home location in radians 
        homeQ = [];

        %> Flag to indicate if tool (such as a gripper) is used
        useTool = false; 

        %> The fule filename and extension of the tool ply filename
        toolFilename = [];

        %> Tool transform (only relevant if there is a tool)
        toolTr = transl(0,0,0);
    end

    properties (Hidden)
        axis_h = [];
        figure_h = [];
        lightAdded = false;
        surfaceAdded = false;
    end

    properties (Access = private)
        delaySecondsForInnerAnimation = 0.2;
        stepsForInnerAnimation = 20;        
    end   
    
    methods (Abstract) % An inheriting class must implement these methods
        % Inherriting class must implement a method which sets
        % self.model to be a SerialLink object
        CreateModel(self);
    end
        
    methods
    
%% General class for multiDOF robot simulation
        function self = RobotBaseClass()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try 
                self.name = [self.plyFileNameStem,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['RobotBaseClass',datestr(now,'yyyymmddTHHMMSSFFF')];
                warning(['Please include a variable called plyFileNameStem in your inherreting class. For now the robot is named: ',self.name])                
            end
            
        end

%% delete
        % Try to clean up gracefully by deleting just this robot plot
        function delete(self)
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
            try delete(h.robot); end 
            try delete(h.wrist); end
            try delete(h.link); end
            try delete(h); end
            try delete(handles);end

            % Cleaning up the axis by removing the tiled floor and lights
            % if the figure and axis still exist
            if ishandle(self.figure_h) && ishandle(self.axis_h)
                % If there are any surface (ground floor) left then delete
                % one if one was initially added
                floor_h = findobj(gca, 'Type', 'surface', 'Tag', 'tiled_floor');
                if self.surfaceAdded
                    try delete(floor_h);end
                end

                light_h = findobj(gca, 'Type', 'light');
                axisChildren_h = get(self.axis_h,'Children');
                % If self added a light and there is now only a single light left then clear (or close) the figure
                if self.lightAdded && size(axisChildren_h,1) == 1 && ~isempty(light_h)
                    % clf % CLEAR
                    delete(self.figure_h); % CLOSE
                end                                             
            end
        end

%% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
        function PlotAndColourRobot(self)
            if isempty(self.homeQ)
                self.homeQ = zeros(1,self.model.n);
            end

            if exist([self.plyFileNameStem,'.mat'],'file') == 2 && exist([self.plyFileNameStem,'Link0.ply'],'file') ~= 2
                warning('There are no ply files for the links but there is a mat file. You should use PlotAndColourRobotMat to create and colour a 3D robot model plot. I am doing this for you now.')
                self.PlotAndColourRobotMat()
                return;
            end

            for linkIndex = 0:self.model.n
                if self.useTool && linkIndex == self.model.n
                    if ~isempty(self.toolFilename)
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.toolFilename],'tri'); %#ok<AGROW>
                    else
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'Tool.ply'],'tri'); %#ok<AGROW>
                    end
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                
                % Obtain faceData and vertexData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            h = self.InitiliseRobotPlot();
            if 1 < length(h)
                self.MultirobotWarningMessage();
                h = h{1};
            end

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;
                    
                catch ME_1
                    disp(ME_1);
                    disp('No vertex colours in plyData');
                    try 
                         vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        disp(['Also, no face colours in plyData, so using a default colour: ',num2str(vertexColours)]);
                    end
                end
                
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
            drawnow();
        end

%% PlotAndColourRobotMat 
% An alternative plot and colour method which uses the .mat file structure
% containing data for the vertex, face and colour data in it 
        function PlotAndColourRobotMat(self)
            % Choose one models with different (or no) end effector
            shapeData = load([self.plyFileNameStem,'.mat']);

            for linkIndex = 0:self.model.n               
                % Obtain faceData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = shapeData.shapeModel(linkIndex+1).face;
                self.model.points{linkIndex+1} = shapeData.shapeModel(linkIndex+1).vertex;
            end
        
            h = self.InitiliseRobotPlot();                   

            if 1 < length(h)
                self.MultirobotWarningMessage();
                h = h{1};
            end

            % Colour the links
            for linkIndex = 0:self.model.n 
                vertexColours = repmat(shapeData.shapeModel(linkIndex+1).colour, size(self.model.points{linkIndex+1}, 1), 1);
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end          
        end    
    end

    methods (Hidden)
%% InitiliseRobotPlot
% First and only time to plot the robot
        function h = InitiliseRobotPlot(self)
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();
            % Display robot
            [ax,by] = view;
            
            roughMinMax = sum(abs(self.model.d) + abs(self.model.a));
            self.workspace = [-roughMinMax roughMinMax -roughMinMax roughMinMax -0.01 roughMinMax]; 

            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by]);%,'notiles');            

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
        end

%% TestMoveJoints      
        % Simple test move of the joints
        function TestMoveJoints(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);
            initialDelay = self.model.delay;
            self.model.delay = self.delaySecondsForInnerAnimation;
            self.model.animate(qPath);
            self.model.delay = initialDelay;
        end

%% TestMoveBase
        % Simple test move of the base
        function TestMoveBase(self)
            startBaseTR = self.model.base.T;
            self.MoveBaseToTR(startBaseTR,transl(-1,-1,0));
            self.MoveBaseToTR(transl(-1,-1,0),transl(1,1,0));
            self.MoveBaseToTR(transl(1,1,0),startBaseTR);
        end

%% MoveBaseToTR 
        % move robot base through a ctraj generated path
        function MoveBaseToTR(self,startTR, endTR)
            trPath = ctraj(startTR,endTR,self.stepsForInnerAnimation);
            for i = 1:size(trPath,3)
                self.model.base = trPath(:,:,i);
                self.model.animate(self.model.getpos);
                pause(self.delaySecondsForInnerAnimation);
            end
        end

%% CountTiledFloorSurfaces
        % A way of checking if a base tiled floor surface has been added
        % that needs deleting
        function surfaceCount = CountTiledFloorSurfaces(self)
            surfaceCount = numel(findobj(self.axis_h, 'Type', 'surface', 'Tag', 'tiled_floor'));
        end

%% MultirobotWarningMessage
        function MultirobotWarningMessage(self)
            disp('There are more than 1 of these robots in the plot although they may be plotted over each other. This shouldnt break anything, but it is a good idea to clean up by deleting identical robots before you replot a new one. For now I will recolour the first (and probably latest) one.')
        end
    end
end