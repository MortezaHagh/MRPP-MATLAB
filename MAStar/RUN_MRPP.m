% MH Multi Robot Path Planning based on Astar - MATLAB
% By Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

% adding paths
addpath('..\common');
addpath('..\models');

%% settings
Model.distType = 'manhattan';  % euclidean manhattan;
Model.adjType = '4adj';        % '4adj' '8adj'

%% create Model
% createModel_mh_1 createModel_mh_2 createModel_mh_ctest
Model = createModel_mh_1(Model);
Model = createModelAstar(Model);

Model.msc = 100;

% % 45r_2
% Model.Robots(26).targetNode = 21;
% Model.Robots(26).xt = 9;
% Model.Robots(26).yt = 1;

robot_count = Model.robot_count;

%% Preallocation
% empty sol char structure
% empChar.path_length = 0;
% empChar.isFeasible = 0;
% empChar.violation = 0;

% empty sol structure
% empS.solChar = empChar;
empS.smoothness = 0;
empS.nodeNumbers = [];
empS.coords = [];
empS.dirs = [];
empS.cost = 0;
empS.len = 0;
empS.x = [];
empS.y = [];
empS.n = 0;

% sol = repmat(empS, robot_count, 1);

%% Astar MRPP
%   MRPP_2    MRPP_1     MRPP_simple
tic
paths = MRPP_2(Model);
pTime = toc;
sol = paths;

for nr = 1:robot_count
    sol(nr).n = nr;
    sol(nr).len = numel(sol(nr).nodeNumbers);
    sol(nr).x = sol(nr).coords(:, 1);
    sol(nr).y = sol(nr).coords(:, 2);
    sol(nr).cost = calCostL(sol(nr).coords);
    %     [sol(nr).cost, sol(nr).solChar] = calCostLinear(Model, sol(nr).coords);
    sol(nr).smoothness = calSmoothness(sol(nr).coords);
end

% sol total cost & smoothness & pTime
total_pTime = toc;
total_cost = sum([sol.cost]);
total_smoothness = sum([sol.smoothness]);
max_operation_time = max([sol.len]);
tot_operation_time = sum([sol.len]);

%% paths collision check
% collision check
collisionsCheck(paths, robot_count)

% disp data
disp(['total_cost: ' num2str(round(total_cost, 2))])
disp(['total_pTime: ' num2str(round(total_pTime, 3))])
disp(['total_smoothness: ' num2str(total_smoothness)])
disp(['max_operation_time: ' num2str(max_operation_time)])
disp(['tot_operation_time: ' num2str(tot_operation_time)])

% for i=1:robot_count
%    disp(sol(i))
% end

% % plot solution
plotModelMulti(Model)
Color = hsv(robot_count);
plotSolution(sol, Model, Color) % plot paths
% plotAnimation_1(sol, Model, Color)    % animated points and paths, in turn.
% plotAnimation_2(sol, Model, Color)      % animated points with tale! concurrent.
% plotAnimation_3(sol, Model, Color)    % animated points and paths, concurrent.

%% clear temporal data
clear i  nr Color temp_time pp_time empS empChar mod_paths distType adjType
