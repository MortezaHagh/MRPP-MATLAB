% MH Multi Robot Path Planning based on Astar - MATLAB
% By Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

% adding paths
addpath('..\common');
addpath('..\models');


% settings
model.dist_type = 'euclidean';          % '4adj'  '8adj'
model.adj_type = '8adj';                % euclidean manhattan;

% create model
% createModel_mh_1 createModel_mh_2 createModel_mh_ctest
model = createModel_mh_ctest(model);
% load m12x12_obst_50_r_35_20; % 14 20
model.msc = 100;

% % 45r_2
% model.robo(26).targetNode = 21;
% model.robo(26).xt = 9;
% model.robo(26).yt = 1;

robot_count = model.robot_count;

% empty sol char structure
% empChar.path_length = 0;
% empChar.isFeasible = 0;
% empChar.violation = 0;

% empty sol structure
% empS.solChar = empChar;
empS.smoothness = 0;
empS.nodes = [];
empS.coords=[];
empS.dirs=[];
empS.cost=0;
empS.len=0;
empS.x=[];
empS.y=[];
empS.n=0;

% sol = repmat(empS, robot_count, 1);
msol= repmat(empS, robot_count, 1);

%% Astar MRPP
%   MRPP_2    MRPP_1     MRPP_simple
tic
paths = MRPP_2(model);
pTime=toc;
sol = paths;
for nr=1:robot_count
    sol(nr).n = nr;
    sol(nr).len = numel(sol(nr).nodes);
    sol(nr).x = sol(nr).coords(:,1);
    sol(nr).y = sol(nr).coords(:,2);
    sol(nr).cost = costL(sol(nr).coords);
    %     [sol(nr).cost, sol(nr).solChar] = costLinear(model, sol(nr).coords);
    sol(nr).smoothness = smoothness(sol(nr).coords);
end

% sol total cost & smoothness & pTime
total_pTime = toc;
total_cost = sum([sol.cost]);
total_smoothness = sum([sol.smoothness]);
max_operation_time = max([sol.len]);
tot_operation_time = sum([sol.len]);

%% paths collision check
collisionsCheck(paths, robot_count)

%% Astar with path modification
% for nr=1:robot_count
%     mpath = modifyPath (model, paths(nr));
%     msol(nr).n = nr;
%     msol(nr).dirs = mpath.dirs;
%     msol(nr).nodes = mpath.nodes;
%     msol(nr).coords = mpath.coords;
%     msol(nr).x = msol(nr).coords(:,1);
%     msol(nr).y = msol(nr).coords(:,2);
%     msol(nr).smoothness = smoothness(msol(nr).coords);
%     [msol(nr).cost, msol(nr).solChar] = costLinear(model, msol(nr).coords);
% end
%
% % Msol total cost & smoothness & pTime
% total_pTime_m = toc;
% total_cost_m = sum([msol.cost]);
% total_smoothness_m = sum([msol.smoothness]);

%% display data & plot solution
% display data
% disp(['total_cost = ' num2str(round(total_cost,2))])
% disp(['total_pTime: ' num2str(round(total_pTime,3))])
% disp(['total_smoothness = ' num2str(total_smoothness)])
% disp(['max operation time = ' num2str(max_operation_time)])
% disp(['tot operation time = ' num2str(tot_operation_time)])

disp(num2str(round(total_cost,2)))
disp(num2str(round(total_pTime,3)))
disp(num2str(total_smoothness))
disp(num2str(max_operation_time))
disp(num2str(tot_operation_time))

% for i=1:robot_count
%    disp(sol(i))
% end

% plot solution
% plotModel(model)
% Color = hsv(robot_count);
% plotSolution(sol, model, Color)      % plot paths
% plotAnimation_1(sol, model, Color)    % animated points and paths, in turn.
% plotAnimation_2(sol, model, Color)      % animated points with tale! concurrent.
% plotAnimation_3(sol, model, Color)    % animated points and paths, concurrent.

%% clear temporal data
clear i  nr Color temp_time pp_time empS empChar mod_paths dist_type adj_type
