%% GC Wireframe Plot Code
%import joint angle kinematic data, segment, and plot wireframe to reflect
%resulting gait pattern

%% load joint angle trajecs
sim_file_dir = '/Users/jess.thayer/Documents/GitHub/GaitSimulation'; %location of simulation directory

n = 1; %number of trials to plot

%simulation1
trajec.m(1).trialName = 'GCSubj04_03-19-2020 10-30-IC01-objfun_v0-inter_v0-plant_v0';
trajec.m(1).name = 'Constrained Simulation - Subject 4';
trajec.m(1).subjID = 'Subject_4';

%simulations
for i = 1:n
    model.m(i).ds1 = load([sim_file_dir '/' trajec.m(i).trialName '/DS1/ds1_plant_end']);
    model.m(i).ss1 = load([sim_file_dir '/' trajec.m(i).trialName '/SS1/ss1_plant_end']);
    %model.m(i).ds2 = load([sim_file_dir '/' trajec.m(i).trialName '/DS2/ds2_plant_end']);
    %model.m(i).ss2 = load([sim_file_dir '/' trajec.m(i).trialName '/SS2/ss2_plant_end']);
    model.m(i).param = load([sim_file_dir '/Param_' trajec.m(i).subjID]);
    model.m(i).shank = model.m(i).param.param.val(13);
    model.m(i).thigh = model.m(i).param.param.val(18);
end

for i = 1:n
    trajec.m(i).stanceankle = [model.m(i).ds1.stance_ankle_p_out; model.m(i).ss1.swing_ankle_p_out];
    trajec.m(i).stanceknee = [-model.m(i).ds1.stance_knee_p_out; model.m(i).ss1.swing_knee_p_out];
    trajec.m(i).stancehip = [model.m(i).ds1.stance_hip_p_out; model.m(i).ss1.swing_hip_p_out];
    trajec.m(i).swingankle = [model.m(i).ds1.swing_ankle_p_out; model.m(i).ss1.stance_ankle_p_out];
    trajec.m(i).swingknee = [-model.m(i).ds1.swing_knee_p_out; model.m(i).ss1.stance_knee_p_out];
    trajec.m(i).swinghip = [model.m(i).ds1.swing_hip_p_out; model.m(i).ss1.stance_hip_p_out];
    trajec.m(i).stancetoex = [model.m(i).ds1.Toe_L_x.Data; model.m(i).ss1.Toe_R_x.Data];
    trajec.m(i).stancetoex = [model.m(i).ds1.Toe_L_x.Data; model.m(i).ss1.Toe_R_x.Data];
    trajec.m(i).stancetoex = [model.m(i).ds1.Toe_L_z.Data; model.m(i).ss1.Toe_R_z.Data];
    trajec.m(i).t = 0.00001*(1:length(trajec.m(i).swinghip));
end

%% interpolate data to create vector for wireframe
for i = 1:n
    xp = linspace(0,trajec.m(i).t(end),20);
    trajec.m(i).norm(:,1) = interp1(trajec.m(i).t,trajec.m(i).stanceankle,xp);
    trajec.m(i).norm(:,2) = interp1(trajec.m(i).t,trajec.m(i).stanceknee,xp);
    trajec.m(i).norm(:,3) = interp1(trajec.m(i).t,trajec.m(i).stancehip,xp);
    trajec.m(i).norm(:,4) = interp1(trajec.m(i).t,trajec.m(i).swingankle,xp);
    trajec.m(i).norm(:,5) = interp1(trajec.m(i).t,trajec.m(i).swingknee,xp);
    trajec.m(i).norm(:,6) = interp1(trajec.m(i).t,trajec.m(i).swinghip,xp);
end
    
