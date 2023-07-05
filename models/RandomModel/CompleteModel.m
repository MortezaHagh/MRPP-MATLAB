clear
clc
close

% add Path
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Models');
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\MRPP');

% settings
Model.distType = 'manhattan';          % euclidean manhattan;
Model.adjType = '4adj';                % '4adj'  '8adj'
Model.occLength = 1000;

% createModel_rand createModel
for nObst=150
    for nRobot=60%10:10:100
        disp(['nRobot ' num2str(nRobot)])
        for i=2:100
            Model.distType = 'manhattan';          % '4adj'  '8adj'
            Model.adjType = '4adj';
            Model = createModelRand(Model, nObst, nRobot);
            disp(['i' num2str(i)])
            % check Feasibility
            isFeasible = checkFeasibility(Model);
            if ~isFeasible
                disp('Not Feasible!')
                continue
            end
            
            name = ['m10x10_Obst_' num2str(nObst) '_r_' num2str(nRobot) '_i_' num2str(i)];
            save(name, 'Model');
            clear Model
        end
    end
end

% % Plot
% plotModelMulti(Model);