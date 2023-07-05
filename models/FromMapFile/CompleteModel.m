clc
clear
close

% adding paths
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Models');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\MRPP');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\SRPP')

% setting
Model.distType = 'manhattan';  % euclidean manhattan;
Model.adjType = '4adj';        % 4adj 8adj

%
Model.robotCount=100;

% create map from file
[Map, Name] = createMap('random-64-64-10.map');

% create model from map
Model = createModelFromMap(Map, Model);

% scenario
scene_path = 'random-64-64-10-random-1.scen';
scenarios = scenarios(scene_path);

load('aa_random-64-64-10-random-1');
scenarios=scenarios(find(aa'),:);

for i=1:10
    % add robot to model
    Model = addRobotToModel(Model, scenarios);
    name = ['random-64-64_r_' num2str(Model.robotCount) '_i_' num2str(i)];
    disp(name)
    save(name, 'Model')
    %     clear Model
end

% % check Feasibility
% isFeasible = checkFeasibility(Model);
% if ~isFeasible
%     disp('Not Feasible!')
% end

% % Plot
% plotModelMulti(Model);
