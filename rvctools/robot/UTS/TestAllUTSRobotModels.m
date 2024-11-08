classdef TestAllUTSRobotModels < handle
    %TestAllUTSRobotModels This is a way to test if all the UTS robot
    % models that are expected to the in this robotics toolbox can be
    % created

%#ok<*AGROW>
    properties
        robotArmList = {'DensoVM6083' ...
                        ,'DensoVS060'...
                        ,'DobotMagician'...
                        ,'Fetch'...
                        ,'HansCute'...
                        ,'IRB120'...
                        ,'KinovaGen2'...
                        ,'KinovaGen3'...
                        ,'LinearUR10'...
                        ,'LinearUR5'...
                        ,'MotomanHC10DTP'...
                        ,'MyCobot280'...
						,'Sawyer'...
                        ,'SchunkUTSv2'...
                        ,'UR10'...
                        ,'UR10(eye(4),true,''DabPrintNozzleTool'')'...
                        ,'UR10e'...                        
                        ,'UR3'...
                        ,'UR3e'...
                        ,'UR5'...
						,'UR5e'};
                otherRobotList = {'ParrotQuadrotorSquad'...
                        ,'Turtlebot3Waffle'...
                        ,'UFOFleet'...                       
                        ,'RobotCows'...
                        ,'uArm'}; % uArm Cannot be fixed easily and may need to be deleted. Does not use SerialLink
            quickTest = false;
    end
    methods
        function self = TestAllUTSRobotModels(quickTest)
            %TestAllUTSRobotModels Construct and run an instance of this
            %class and run the tests for all robots in the list
			
			if nargin < 1
			    self.quickTest = false;
            else
                self.quickTest = quickTest;
			end
			
            % self.LoadEachRobotArmOnce();
            % self.LoadEachOtherRobotOnce();
            % profile clear; profile on;
            self.LoadAllRobotArmsTogether();
            % profile off; profile viewer;
        end

        %% LoadEachRobotOnce
        function LoadEachRobotArmOnce(self)
            disp('--- Running LoadEachRobotOnce ----')
            clf
            for i = 1:size(self.robotArmList,2)
                % Try create and plot the robot
                disp(['Testing ',self.robotArmList{i}]);
                try r = eval(self.robotArmList{i});
                catch ME_1
                    disp(ME_1);
                    warning(['Could not create ',self.robotArmList{i},', so moving to the next robot']);
                    if self.quickTest
                        keyboard
                    end
                    continue
                end
				
				if self.quickTest
					disp('Quick test so not moving the robot. Continuing to the next robot.');
					continue
				end
                % Try and move the robot's joints and base
                try 
                    title(['Test ',num2str(i),' of ',num2str(size(self.robotArmList,2)),': ',self.robotArmList{i}]);
                    drawnow();
                    pause(2);
                    r.TestMoveJoints();
                    r.TestMoveBase();
                catch ME_1
                    disp(ME_1);
                end

                % Try delete the robot
                try delete(r);
                catch ME_1
                    disp(ME_1);
                    warning(['Could not delete ',self.robotArmList{i}]);
                end
            end
            disp('--- Finished LoadEachRobotOnce ----')
        end

%% LoadEachOtherRobotOnce
        function LoadEachOtherRobotOnce(self)
            disp('--- Running LoadEachOtherRobotOnce ----')
            clf
            for i = 1:size(self.otherRobotList,2)
                % Try create and plot the robot
                disp(['Testing ',self.otherRobotList{i}]);
                try r = eval(self.otherRobotList{i});
                catch ME_1
                    disp(ME_1);
                    warning(['Could not create ',self.otherRobotList{i},', so moving to the next robot']);
                    if self.quickTest
                        keyboard
                    end
                    continue
                end
                                
                % Try delete the robot
                try delete(r);
                catch ME_1
                    disp(ME_1);
                    warning(['Could not delete ',self.robotArmList{i}]);
                end
            end
            disp('--- Finished LoadEachOtherRobotOnce ----')
        end        
        

%% LoadAllRobotArmsTogether
        function LoadAllRobotArmsTogether(self)
            disp('--- Running LoadAllRobotsTogether ----')            
            clf
            hold on;
            robotCount = size(self.robotArmList,2);
            width = floor(sqrt(robotCount));
            length = ceil(sqrt(robotCount));

            robotIndex = 0;
            for x = -width/2:1:width/2
                for y = -length/2:1:length/2
                    robotIndex = robotIndex + 1;
                    if robotCount < robotIndex
                        break;
                    end
                    % Try create and plot the robot
                    disp(['Loading ',self.robotArmList{robotIndex}]);
                    try 
                        if isempty(strfind(self.robotArmList{robotIndex},'('))
                            r{robotIndex} = eval([self.robotArmList{robotIndex},'(transl(x,y,0))']); 
                        else
                            disp('Since there are already variables passed to this robot in the robotArmList, attempting to load and then set the base tr.');
                            r{robotIndex} = eval([self.robotArmList{robotIndex}]);
                            r{robotIndex}.model.base = transl(x,y,0);
                            r{robotIndex}.model.animate(r{robotIndex}.model.getpos)
                        end
                        axis equal;
                        view(3);
                    catch ME_1
                        disp(ME_1);
                        warning(['Could not create ',self.robotArmList{robotIndex},', so moving to the next robot']);                        
                        if self.quickTest
                            keyboard
                        end
                        continue
                    end
                end
            end
            disp('--- Finished LoadAllRobotsTogether ----');
        end
    end
end