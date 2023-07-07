% MH Multi Robot Path Planning based on Astar - MATLAB
% By Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

% adding Paths
addpath('..\common');
addpath('..\models');

%% settings
Model.distType = 'manhattan';  % euclidean manhattan;
Model.adjType = '4adj';        % '4adj' '8adj'

%% create Model
% createModel_1 createModel_2 createModel_ctest
Model = createModel_1(Model);
Model = createModelAstar(Model);

Model.msc = 100;

% % 45r_2
% Model.Robots(26).targetNode = 21;
% Model.Robots(26).xt = 9;
% Model.Robots(26).yt = 1;

robotCount = Model.robotCount;

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

% sol = repmat(empS, robotCount, 1);

%% Astar MRPP
%   MRPP_2    MRPP_1     MRPP_simple
tic
Paths = MRPP_2(Model);
pTime = toc;
sol = Paths;

for nr = 1:robotCount
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

%% Paths collision check
% collision check
collisionsCheck(Paths, robotCount)

% disp data
disp(['total_cost: ' num2str(round(total_cost, 2))])
disp(['total_pTime: ' num2str(round(total_pTime, 3))])
disp(['total_smoothness: ' num2str(total_smoothness)])
disp(['max_operation_time: ' num2str(max_operation_time)])
disp(['tot_operation_time: ' num2str(tot_operation_time)])

% for i=1:robotCount
%    disp(sol(i))
% end

% % plot solution
plotModelMulti(Model)
Color = hsv(robotCount);
plotSolution(sol, Model, Color) % plot Paths
% plotAnimation_1(sol, Model, Color)    % animated points and Paths, in turn.
% plotAnimation_2(sol, Model, Color)      % animated points with tale! concurrent.
% plotAnimation_3(sol, Model, Color)    % animated points and Paths, concurrent.

%% clear temporal data
clear i  nr Color temp_time pp_time empS empChar mod_paths distType adjType
