classdef ParrotQuadrotorSquad < handle
    %ParrotQuadrotor A way of creating a group of ParrotQuadrotors
    %   The uavs can be moved around randomly. It is then possible to query
    %   the current location (base) of the uav.
       
    properties (Constant)                
        %> flightVolume in meters
        flightVolume = [10,10,10]; 
    end
    
    properties
        %> Number of ParrotQuadrotor
        count = 2;
        
        %> A cell structure of \c count models
        uav;
               
        %> Dimensions of the workspace in regard to the flightVolume
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = ParrotQuadrotorSquad(count)
            if 0 < nargin
                self.count = count;
            end
            
            self.workspaceDimensions = [-self.flightVolume(1)/2, self.flightVolume(1)/2 ...
                                       ,-self.flightVolume(2)/2, self.flightVolume(2)/2 ...
                                       ,-self.flightVolume(3)/2, self.flightVolume(3)/2];

            % Create the required number of UAVs
            for i = 1:self.count
                self.uav{i} = self.GetModel(['uav',num2str(i)]);
                % Random spawn
                self.uav{i}.base = SE3(SE2((2 * rand()-1) * self.flightVolume(1)/2 ...
                                         , (2 * rand()-1) * self.flightVolume(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi));
                 % Plot 3D model
                plot3d(self.uav{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            camlight;
        end
        
        function delete(self)
%             cla;
        end       
        
        %% PlotSingleRandomStep
        % Move each of the UAVs forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for index = 1:self.count
                % Move Forward
                self.uav{index}.base = self.uav{index}.base * SE3(SE2(0.2, 0, 0));
                animate(self.uav{index},0);
                % Move Up (or down)
                self.uav{index}.base = self.uav{index}.base.T *  transl(0,0, (rand - 0.5) * 0.2);
                animate(self.uav{index},0);
                
                % Turn randomly
                % Save base as a temp variable
                tempBase = self.uav{index}.base.T;
                rotBase = tempBase(1:3, 1:3);
                posBase = tempBase(1:3, 4);
                newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                newBase = [newRotBase posBase ; zeros(1,3) 1];
                
                % Update base pose
                self.uav{index}.base = newBase;
                animate(self.uav{index},0);                 

                % If outside workspace rotate back around
                % Get base as temp
                tempBase = self.uav{index}.base.T;
                if tempBase(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < tempBase(1,4) ...
                || tempBase(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < tempBase(2,4) ...
                || tempBase(3,4) < self.workspaceDimensions(5) ...
                || self.workspaceDimensions(6) < tempBase(3,4)
                    self.uav{index}.base = self.uav{index}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                end
                
                % Move up since too low
                % Get base as temp
                tempBase = self.uav{index}.base.T;
                if tempBase(3,4) < self.workspaceDimensions(5)
                    self.uav{index}.base = tempBase * transl(0,0,0.2);
                end

                % Move down since too high
                % Get base as temp
                tempBase = self.uav{index}.base.T;
                if self.workspaceDimensions(6) < tempBase(3,4)
                    self.uav{index}.base = tempBase * transl(0,0,-0.2);
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetModel
        function model = GetModel(name)
            if nargin < 1
                name = 'ParrotQuadrotor';
            end
            [faceData,vertexData,plyData] = plyread('Parrot_ARDrone_Quadrotor.ply','tri');
            link(1) = Link('alpha',0,'a',1,'d',0,'offset',0);
            model = SerialLink(link,'name',name);
            % Changing order of cell array from {self.faceData, []} to 
            % {[], self.vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};
            vertexData(:,2) = vertexData(:,2);
            % Changing order of cell array from {self.faceData, []} to 
            % {[], self.vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end

