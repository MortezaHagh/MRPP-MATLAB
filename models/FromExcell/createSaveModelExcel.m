clc
clear
close

% add path
addpath('..\..\common');

% settings
Model.distType = 'manhattan'; % euclidean manhattan;
Model.adjType = '4adj'; % '4adj' '8adj'
Model.occLength = 1000;

% from Excel
Model = createModelFromExcel(Model);

% % check Feasibility
% isFeasible = checkFeasibility(Model);
% if ~isFeasible
%     disp('Not Feasible!')
% end

% % save model
% save('Model','Model')

% Plot
plotModelMulti(Model);
