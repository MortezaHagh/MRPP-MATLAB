clc
clear
close

% add path
addpath('..\..\common');

% setting
Model.distType = 'manhattan'; % euclidean manhattan;
Model.adjType = '4adj'; % 4adj 8adj

%
Model.robotCount = 10;

% create map from file
[Map, Name] = createMapFromFile('random-64-64-10.map');

% create Model from map
Model = createModelFromMap(Map, Model);

% scenario
scene_path = 'random-64-64-10-random-1.scen';
scenarios = createScenarios(scene_path);

load('aa_random-64-64-10-random-1');
scenarios = scenarios(find(aa'), :);

N = 1;

for i = 1:N
    % add robot to Model
    Model = addRobotToModel(Model, scenarios);
    name = ['random-64-64_r_' num2str(Model.robotCount) '_i_' num2str(i)];
    disp(name)
    % save(name, 'Model')
    % clear Model
end

% % check Feasibility
% isFeasible = checkFeasibility(Model);
% if ~isFeasible
%     disp('Not Feasible!')
% end

% Plot
plotModelMulti(Model);
