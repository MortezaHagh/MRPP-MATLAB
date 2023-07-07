% MH D* Based Multi Robot Path Planning - MATLAB
% Main code for running the algorithm.
% Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

% adding paths
addpath('..\common');
addpath('..\models');

%% settings
Model.expandMethod = 'heading';  % random or heading
Model.distType = 'manhattan';    % euclidean or manhattan;
Model.adjType = '4adj';          % 4adj or 8adj

% number of occupant nodes on fined path
Model.occLength = 1000;

%% create Model

% from function createModel_mrdsl_Article createModel_mrdsl_2
Model = createModel_mrdsl_1(Model);

% robotCount
robotCount = Model.robotCount;

%% check Feasibility
% isFeasible = checkFeasibility(Model);

%% MRPP
tic
[Model, Robots, Paths] = myMRDstarLite(Model);
run_time=toc;
Sol = Paths;
for iRobot=1:robotCount
    Sol(iRobot).x = Sol(iRobot).coords(:,1);
    Sol(iRobot).y = Sol(iRobot).coords(:,2);
    Sol(iRobot).cost = calCostL(Sol(iRobot).coords);
    Sol(iRobot).len = numel(Sol(iRobot).nodeNumbers);
    Sol(iRobot).makespan = numel(Sol(iRobot).nodeNumbers);
    Sol(iRobot).smoothness = calSmoothnessByDir(Sol(iRobot));
    
end

% sol total cost & smoothness & pTime
total_runTime = run_time;
makespan = max([Sol.len]);
total_cost = sum([Sol.cost]);
sum_of_costs = sum([Sol.len]);
total_smoothness = sum([Sol.smoothness]);

%% paths collision check
collisionsCheck(Paths, robotCount)

%% display data & plot solution
% display data
disp(['makespan = ' num2str(makespan)])
disp(['sum_of_costs = ' num2str(sum_of_costs)])
disp(['total_cost = ' num2str(round(total_cost,2))])
disp(['total_smoothness = ' num2str(total_smoothness)])
disp(['total_runTime: ' num2str(round(total_runTime,3))])

% plot solution
Color = hsv(robotCount);
plotModelMulti(Model)
plotSolution(Sol, Model, Color)      % plot paths
% plotAnimation_1(Sol, Model, Color)    % animated points and paths, in turn.
% plotAnimation_2(Sol, Model, Color)      % animated points with tale! concurrent.
% plotAnimation_3(Sol, Model, Color)    % animated points and paths, concurrent.

%% clear temporal data
clear i  nr Color temp_time pp_time empS empChar mod_paths dist_type adj_type
