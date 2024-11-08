classdef TestLOSCoverage < handle
    %% From file CSH_B1_W_L_v1.lay
            % [points]
            % -2 = (2.9210702, -2.8939985999999998)
            % -3 = (7.634180100000001, -2.8939985999999998)
            % -4 = (7.634180100000001, -1.4800685999999998)
            % -5 = (10.46205, 1.3478013)
            % -6 = (36.855498, 1.3478013)
            % -7 = (39.683358, -1.4800686)
            % -8 = (39.683358, -2.8939985999999998)
            % -9 = (146.19974, -2.8939985999999998)
            % -10 = (146.19974, 25.856001)
            % -11 = (140.54401, 25.856001)
            % -12 = (138.18744999999998, 25.384691)
            % -13 = (39.683358, 25.384691)
            % -14 = (36.384178, 22.085511)
            % -15 = (10.46205, 22.085511)
            % -16 = (7.6341801, 24.913380999999998)
            % -17 = (2.9210702, 24.913380999999998)
            % -18 = (38.272002, 8.5760003)
            % -19 = (43.648002, 8.5760003)
            % -20 = (43.648002, 14.336001)
            % -21 = (38.272002, 14.336001)
            % -22 = (50.560001, 8.5760003)
            % -23 = (55.936001, 8.5760003)
            % -24 = (55.936001, 14.336001)
            % -25 = (50.560001, 14.336001)
            % 
            % [segments]
            % 1 = {'name': 'WALL', 'connect': [-2, -3], 'z': (0, 3)}
            % 2 = {'name': 'WALL', 'connect': [-3, -4], 'z': (0, 3)}
            % 3 = {'name': 'WALL', 'connect': [-4, -5], 'z': (0, 3)}
            % 4 = {'name': 'WALL', 'connect': [-5, -6], 'z': (0, 3)}
            % 5 = {'name': 'WALL', 'connect': [-6, -7], 'z': (0, 3)}
            % 6 = {'name': 'WALL', 'connect': [-7, -8], 'z': (0, 3)}
            % 7 = {'name': 'WALL', 'connect': [-8, -9], 'z': (0, 3)}
            % 8 = {'name': 'WALL', 'connect': [-9, -10], 'z': (0, 3)}
            % 9 = {'name': 'WALL', 'connect': [-10, -11], 'z': (0, 3)}
            % 10 = {'name': 'WALL', 'connect': [-11, -12], 'z': (0, 3)}
            % 11 = {'name': 'WALL', 'connect': [-12, -13], 'z': (0, 3)}
            % 12 = {'name': 'WALL', 'connect': [-13, -14], 'z': (0, 3)}
            % 13 = {'name': 'WALL', 'connect': [-14, -15], 'z': (0, 3)}
            % 14 = {'name': 'WALL', 'connect': [-15, -16], 'z': (0, 3)}
            % 15 = {'name': 'WALL', 'connect': [-16, -17], 'z': (0, 3)}
            % 16 = {'name': 'WALL', 'connect': [-17, -2], 'z': (0, 3)}
            % 17 = {'name': 'WINDOW_GLASS', 'connect': [-18, -19], 'z': (0, 3)}
            % 18 = {'name': 'WINDOW_GLASS', 'connect': [-19, -20], 'z': (0, 3)}
            % 19 = {'name': 'WINDOW_GLASS', 'connect': [-20, -21], 'z': (0, 3)}
            % 20 = {'name': 'WINDOW_GLASS', 'connect': [-21, -18], 'z': (0, 3)}
            % 21 = {'name': 'WINDOW_GLASS', 'connect': [-22, -23], 'z': (0, 3)}
            % 22 = {'name': 'WINDOW_GLASS', 'connect': [-23, -24], 'z': (0, 3)}
            % 23 = {'name': 'WINDOW_GLASS', 'connect': [-24, -25], 'z': (0, 3)}
            % 24 = {'name': 'WINDOW_GLASS', 'connect': [-25, -22], 'z': (0, 3)}

    properties (Constant)
        pointsFloor = [2.9210702, -2.8939985999999998,0;
                        7.634180100000001, -2.8939985999999998,0;
                        7.634180100000001, -1.4800685999999998,0;
                        10.46205, 1.3478013,0;
                        36.855498, 1.3478013,0;
                        39.683358, -1.4800686,0;
                        39.683358, -2.8939985999999998,0;
                        146.19974, -2.8939985999999998,0;
                         146.19974, 25.856001,0;
                         140.54401, 25.856001,0;
                         138.18744999999998, 25.384691,0;
                         39.683358, 25.384691,0;
                         36.384178, 22.085511,0;
                         10.46205, 22.085511,0;
                         7.6341801, 24.913380999999998,0;
                         2.9210702, 24.913380999999998,0;
                         38.272002, 8.5760003,0;
                         43.648002, 8.5760003,0;
                         43.648002, 14.336001,0;
                         38.272002, 14.336001,0;
                         50.560001, 8.5760003,0;
                         55.936001, 8.5760003,0;
                         55.936001, 14.336001,0;
                         50.560001, 14.336001,0];
    end
    properties
         pointCeiling = TestLOSCoverage.pointsFloor + [0,0,3];                         

         %> This defines quads by indexing into both the pointCeiling
         %and pointFloor. So [index1,index2] would mean a quad that connects
         %index pointCeiling(index1,:),pointCeiling(index2,:),pointFloor(index1,:),pointFloor(index2,:)
        quadFaces = [1     2
                     2     3
                     3     4
                     4     5
                     5     6
                     6     7
                     7     8
                     8     9
                     9    10
                    10    11
                    11    12
                    12    13
                    13    14
                    14    15
                    15    16
                    16     1
                    17    18
                    18    19
                    19    20
                    20    17
                    21    22
                    22    23
                    23    24
                    24    21];  

        vertices = [];
        triFaces = [];
        minPointsFloor = [];
        maxPointsCeiling = [];

        %> This will run without the roolbox, but is better with it
        roboticsToolboxInstalled = true;
    end

    methods
        function self = TestLOSCoverage()
            pointsFloorCount = size(self.pointsFloor,1);
            pointsCeilingCount = size(self.pointCeiling,1);
            assert(pointsFloorCount == pointsCeilingCount);
            
            self.minPointsFloor = min(self.pointsFloor);
            self.maxPointsCeiling = max(self.pointsFloor);

            self.vertices = [self.pointsFloor; self.pointCeiling];
            
            % This makes the quad faces into two triangles. Calcs would be almost twice as
            % fast to do it as quads since we would need to do half as many
            % intersection calculations, but it is not done... yet 
            self.triFaces = zeros(2*size(self.quadFaces,1),3);
            for i = 1:size(self.quadFaces,1)
                self.triFaces(2*(i-1)+1:2*i,:) = [self.quadFaces(i,:), pointsFloorCount + self.quadFaces(i,1)...
                                              ;pointsFloorCount + self.quadFaces(i,:), self.quadFaces(i,2)];    
            end
            
            clf

            % Plot the triangles
            patch('Vertices',self.vertices,'Faces',self.triFaces,'FaceColor','red','EdgeColor','black');
            xlabel('x'); ylabel('y'); zlabel('z');
            axis equal; 
            view(3)
            hold on;
            camlight;

            %% Run Test cases
            self.GetFinelySampledPointsOnWalls();
            self.GridSampledIdenticalTxRx();
            self.RandomTxWalk();
            self.ProfileGridSampledIdenticalTxRx();
        end

%% GetFinelySampledPointsOnWalls        
        function GetFinelySampledPointsOnWalls(self)
            % Work out a sampling grid based on the above vertices (note that some
            % points will still be outside the map)
            samplingMeters = 2;
            [rxX,rxY] = meshgrid(floor(self.minPointsFloor(1))-samplingMeters:samplingMeters:ceil(self.maxPointsCeiling(1)+samplingMeters) ....
                            ,floor(self.minPointsFloor(2))-samplingMeters:samplingMeters:ceil(self.maxPointsCeiling(2)+samplingMeters));    
        
            pointCount = size(rxX(:),1);
            rxPointSet = [rxX(:), rxY(:), repmat(1.5,pointCount,1)];
        
            %% Mega list of point to point
            txPointSet = zeros(pointCount * pointCount,3);
            for i = 1:pointCount
                txPointSet((i-1)*pointCount+1:i*pointCount,:) = repmat(rxPointSet(i,:),pointCount,1);
            end
            rxPointSetOriginal = rxPointSet;
            rxPointSet = repmat(rxPointSetOriginal,pointCount,1);
        
            % Go through each tx position one at a time (i.e. tx are in set that
            % are pointCount long and they are all the same, where as the rx are
            % all different and in repeating groups of pointCount long 
            intersectionPoints = [];
            for i = 1:pointCount
                txPointSubSet = txPointSet((i-1)*pointCount+1:i*pointCount,:); 
                rxPointSubSet = rxPointSet((i-1)*pointCount+1:i*pointCount,:);
                
                intersectionPoints = [intersectionPoints; LinesTriangleIntersection(self.vertices,self.triFaces,txPointSubSet,rxPointSubSet)];  %#ok<AGROW> % Cannot know the full size 
            end
            
            % Plot the points of intersection
            plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');
            wallRxMountingPoints = unique(round(intersectionPoints*1000)/1000,'rows');
            save('wallRxMountingPoints.mat','wallRxMountingPoints');
        end

%% GridSampledIdenticalTxRx        
        function GridSampledIdenticalTxRx(self)               
            % Work out a sampling grid based on the above vertices (note that some
            % points will still be outside the map)
            samplingMeters = 2;
            [rxX,rxY] = meshgrid(floor(self.minPointsFloor(1)):samplingMeters:ceil(self.maxPointsCeiling(1)) ....
                                ,floor(self.minPointsFloor(2)):samplingMeters:ceil(self.maxPointsCeiling(2)));       
        
            pointCount = size(rxX(:),1);
            rxPointSet = [rxX(:), rxY(:), repmat(1.5,pointCount,1)];
        
            %% Mega list of point to point
            txPointSet = zeros(pointCount * pointCount,3);
            for i = 1:pointCount
                txPointSet((i-1)*pointCount+1:i*pointCount,:) = repmat(rxPointSet(i,:),pointCount,1);
            end
            rxPointSetOriginal = rxPointSet;
            rxPointSet = repmat(rxPointSetOriginal,pointCount,1);
            
            txPointCoverage = zeros(pointCount,1);
        
            profile clear; profile on;
            % Go through each tx position one at a time (i.e. tx are in set that
            % are pointCount long and they are all the same, where as the rx are
            % all different and in repeating groups of pointCount long 
            for i = 1:pointCount
                txPointSubSet = txPointSet((i-1)*pointCount+1:i*pointCount,:); 
                rxPointSubSet = rxPointSet((i-1)*pointCount+1:i*pointCount,:);
                
                    %% LOS Coverage Calcs
                    [losCoverageLineIndex] = LOSCoverage(self.vertices,self.triFaces,txPointSubSet,rxPointSubSet);
                    txPointCoverage(i) = sum(losCoverageLineIndex);
                
                    %% Plotting   
                    % try delete(txPointPlot_h);end
                    % try delete(rxPointPlot_h);end                        
                    % txPointPlot_h = plot3(txPointSubSet(:,1),txPointSubSet(:,2),txPointSubSet(:,3),'r*','MarkerSize',20);
                    % rxPointPlot_h = plot3(rxPointSubSet(losCoverageLineIndex,1),rxPointSubSet(losCoverageLineIndex,2),rxPointSubSet(losCoverageLineIndex,3),'b.');
                    % drawnow();
                    
                    %% Early break   
                    % if i == 1000
                    %     break;
                    % end
            end
                             
            % find(txPointSet(1:pointCount:end,:)~=rxPointSet(1:pointCount,:),1);          
            scatter3(txPointSet(1:pointCount:end,1), txPointSet(1:pointCount:end,2), txPointSet(1:pointCount:end,3), 20, txPointCoverage, 'filled');        
            title('For a Tx at this spot it has LOS coverage to this many Rx')
            colorbar
        end

%% ProfileGridSampledIdenticalTxRx        
        function ProfileGridSampledIdenticalTxRx(self,maxCount)
            if nargin < 2
                maxCount = 1000; % it won't necessarily get here
            end

            % Work out a sampling grid based on the above vertices (note that some
            % points will still be outside the map)
            samplingMeters = 3;
            [rxX,rxY] = meshgrid(floor(self.minPointsFloor(1)):samplingMeters:ceil(self.maxPointsCeiling(1)) ....
                                ,floor(self.minPointsFloor(2)):samplingMeters:ceil(self.maxPointsCeiling(2)));                
            pointCount = size(rxX(:),1);
            rxPointSet = [rxX(:), rxY(:), repmat(1.5,pointCount,1)];
        
            %% Mega list of point to point
            txPointSet = zeros(pointCount * pointCount,3);
            for i = 1:pointCount
                txPointSet((i-1)*pointCount+1:i*pointCount,:) = repmat(rxPointSet(i,:),pointCount,1);
            end
            rxPointSetOriginal = rxPointSet;
            rxPointSet = repmat(rxPointSetOriginal,pointCount,1);
            
            txPointCoverage = zeros(pointCount,1);
        
            profile clear; profile on;
            % Go through each tx position one at a time (i.e. tx are in set that
            % are pointCount long and they are all the same, where as the rx are
            % all different and in repeating groups of pointCount long             
            for i = 1:pointCount
                txPointSubSet = txPointSet((i-1)*pointCount+1:i*pointCount,:); 
                rxPointSubSet = rxPointSet((i-1)*pointCount+1:i*pointCount,:);
                
                    %% LOS Coverage Calcs
                    % profile clear; profile on;
                    [losCoverageLineIndex] = LOSCoverage(self.vertices,self.triFaces,txPointSubSet,rxPointSubSet);
                    txPointCoverage(i) = sum(losCoverageLineIndex);
                    % profile off; profile viewer;
                
                    %% Early break   
                    if i == maxCount
                        break;
                    end
            end                       
            profile off; profile viewer;
        end

%% RandomTxWalk        
% Random walk tx and showing LOS coverage
        function RandomTxWalk(self)
            samplingMeters = 3;
            if self.roboticsToolboxInstalled
                personPlot_h = PlaceObject('personBuilder.ply');
                axis([2.9211  146.1997   -2.8940   25.8560   -10 30]);
            end

            [rxX,rxY] = meshgrid(floor(self.minPointsFloor(1)):samplingMeters:ceil(self.maxPointsCeiling(1)) ....
                                ,floor(self.minPointsFloor(2)):samplingMeters:ceil(self.maxPointsCeiling(2)));                
            pointCount = size(rxX(:),1);
            rxPointSet = [rxX(:), rxY(:), repmat(1.5,pointCount,1)];

            txPoint = [70,10,1];  
            for i = 1:500
                txPoint(1:2) = txPoint(1:2) + 2*(rand(1,2)-0.5); % Random walk
                txPointSet = repmat(txPoint,pointCount,1);           
        
            %% Plotting                
                if self.roboticsToolboxInstalled
                    TransformMesh(personPlot_h,transl(txPoint));
                end
                try delete(txPointPlot_h);end
                txPointPlot_h = plot3(txPointSet(:,1),txPointSet(:,2),txPointSet(:,3)+2,'r*','MarkerSize',20);

                drawnow();

                %% LOS Coverage Calcs
                % profile clear; profile on;
                [losCoverageLineIndex] = LOSCoverage(self.vertices,self.triFaces,txPointSet,rxPointSet);
                % profile off; profile viewer;

                try delete(rxPointPlot_h);end
                rxPointPlot_h = plot3(rxPointSet(losCoverageLineIndex,1),rxPointSet(losCoverageLineIndex,2),rxPointSet(losCoverageLineIndex,3),'b.');
                drawnow();
            end    
        end          
    end
end