clear
clc
close


% add path
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Models');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\MRPP');

% settings
Model.distType = 'manhattan';          % euclidean manhattan;
Model.adjType = '4adj';                % '4adj'  '8adj'
Model.occLength = 1000;

% from Excel
Model = createModelFromExcel(Model);

% % check Feasibility
% isFeasible = checkFeasibility(Model);
% if ~isFeasible
%     disp('Not Feasible!')
% end

% Plot
plotModelMulti(Model);
