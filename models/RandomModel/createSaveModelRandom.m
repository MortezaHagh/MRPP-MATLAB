clear
clc
close

% add path
addpath('..\..\common');

% settings
Model.distType = 'manhattan'; % euclidean manhattan;
Model.adjType = '4adj'; % '4adj' '8adj'
Model.occLength = 1000;

% createModel_rand createModel
for nObst = 100

    for nRobot = 60 % 10:10:100
        disp(['nRobot ' num2str(nRobot)])

        for i = 1 % 2:100
            Model.distType = 'manhattan'; % '4adj' '8adj'
            Model.adjType = '4adj';
            Model = createModelRand(Model, nObst, nRobot);
            disp(['i' num2str(i)])

            %             % check Feasibility
            %             isFeasible = checkFeasibility(Model);
            %             if ~isFeasible
            %                 disp('Not Feasible!')
            %                 continue
            %             end

            %             name = ['m10x10_Obst_' num2str(nObst) '_r_' num2str(nRobot) '_i_' num2str(i)];
            %             save(name, 'Model');
            %             clear Model
        end

    end

end

% Plot
plotModelMulti(Model);
