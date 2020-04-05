%% DS 1
global directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc

cd([sim_file_dir '\' trialName]) %check directory location before creating new folder

mkdir('DS1') %create a folder for gait period
subdirectory = cd('DS1'); %open new folder

%% begin double support 1
delete(gcp('nocreate'))

diary on % Record the history of the command window
diary DSCommandWindowHistory.txt

tic % Start a timer to record how long optimization takes

% Start the parallel computing
parpool

% Open human gait plant model and internal MPC models
open_system(ds_inter_loc);
save_system(ds_inter_loc, 'ds1_inter');
open_system(ds_plant_loc);
save_system(ds_plant_loc, 'ds1_plant');

% Current time is set to be 0
tnow = 0;

% Open internal model, initial conditions inherited
sys = 'ds1_inter';
open_system(sys);

% Get the design variables for the MPC optimization
p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagStanceKnee','lagSwingAnkle','lagSwingKnee',...
    'w_stance_ankle_i', 'w_stance_knee_i','w_stance_hip_i','w_swing_knee_i','w_swing_hip_i'});

% Set the maximum and minimum allowable values of the design variables
p(1).Minimum = -150*ones(1,6);
p(2).Minimum = -30*ones(1,6);
p(3).Minimum = -8*ones(1,6);
p(4).Minimum = -15*ones(1,6);
p(1).Maximum = 150*ones(1,6);
p(2).Maximum = 30*ones(1,6);
p(3).Maximum = 8*ones(1,6);
p(4).Maximum = 15*ones(1,6);
temp1 = p(5).Value*0.9;
temp2 = p(5).Value*1.1;
if temp1 < temp2
    p(5).Minimum = temp1;
    p(5).Maximum = temp2;
else
    p(5).Minimum = temp2;
    p(5).Maximum = temp1;
end
temp1 = p(6).Value*0.9;
temp2 = p(6).Value*1.1;
if temp1 < temp2
    p(6).Minimum = temp1;
    p(6).Maximum = temp2;
else
    p(6).Minimum = temp2;
    p(6).Maximum = temp1;
end
temp1 = p(7).Value*0.9;
temp2 = p(7).Value*1.1;
if temp1 < temp2
    p(7).Minimum = temp1;
    p(7).Maximum = temp2;
else
    p(7).Minimum = temp2;
    p(7).Maximum = temp1;
end
temp1 = p(8).Value*0.9;
temp2 = p(8).Value*1.1;
if temp1 < temp2
    p(8).Minimum = temp1;
    p(8).Maximum = temp2;
else
    p(8).Minimum = temp2;
    p(8).Maximum = temp1;
end
temp1 = p(9).Value*0.9;
temp2 = p(9).Value*1.1;
if temp1 < temp2
    p(9).Minimum = temp1;
    p(9).Maximum = temp2;
else
    p(9).Minimum = temp2;
    p(9).Maximum = temp1;
end

% Set the scale of the design variables
p(1).Scale = 150*ones(1,6);
p(2).Scale = 30*ones(1,6);
p(3).Scale = 8*ones(1,6);
p(4).Scale = 15*ones(1,6);
p(5).Scale = abs(p(5).Value);
p(6).Scale = abs(p(6).Value);
p(7).Scale = abs(p(7).Value);
p(8).Scale = abs(p(8).Value);
p(9).Scale = abs(p(9).Value);

% Create a simulation tester
simulator = sdo.SimulationTest(sys);

% Define the optimization algorithm
addpath(sim_file_dir);
costfxn = str2func(['objfunDS_' objfun_vers]);
evalDesign = @(p) costfxn(p,simulator);

% Select options for the optimizer
opt = sdo.OptimizeOptions;
opt.MethodOptions.Algorithm = 'interior-point';
opt.MethodOptions.UseParallel = 'always';
%opt.MethodOptions.DiffMaxChange = 4;
%opt.MethodOptions.TypicalX = [];
opt.MethodOptions.FinDiffType = 'central';
opt.MethodOptions.Hessian = 'bfgs';
opt.MethodOptions.TolFun = 0.1;
opt.MethodOptions.ObjectiveLimit = 5;
opt.MethodOptions.MaxIter = 20;
opt.MethodOptions.TolX = 0.02;
opt.UseParallel = 'always';
opt.OptimizedModel = sys;

% Start the optimization
[pOpt,optInfo] = sdo.optimize(evalDesign,p,opt);

% Save the optimization results of the first iteration
save('ds1_inter_iter_1');

for i = 1:9
    u(i,:) = pOpt(i).Value;
end

save('ds1_optimizedcontrolinput_iter_1','u');

% Load the human gait anthropometric data and the optimized control input
Plant = 'ds1_plant';

%send optimized control input to plant
sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
sdo.setValueInModel(Plant,'u_stance_knee',u(2,:));
sdo.setValueInModel(Plant,'u_swing_ankle',u(3,:));
sdo.setValueInModel(Plant,'u_swing_knee',u(4,:));
sdo.setValueInModel(Plant,'w_stance_ankle',u(5));
sdo.setValueInModel(Plant,'w_stance_knee',u(6));
sdo.setValueInModel(Plant,'w_stance_hip',u(7));
sdo.setValueInModel(Plant,'w_swing_knee',u(8));
sdo.setValueInModel(Plant,'w_swing_hip',u(9));

u_stance_ankle_data_timeseries = timeseries;
u_stance_ankle_data_timeseries.Time = 0;
u_stance_ankle_data_timeseries.Data = 0;
u_stance_knee_data_timeseries = timeseries;
u_stance_knee_data_timeseries.Time = 0;
u_stance_knee_data_timeseries.Data = 0;
u_swing_ankle_data_timeseries = timeseries;
u_swing_ankle_data_timeseries.Time = 0;
u_swing_ankle_data_timeseries.Data = 0;
u_swing_knee_data_timeseries = timeseries;
u_swing_knee_data_timeseries.Time = 0;
u_swing_knee_data_timeseries.Data = 0;

clear u;

%simulate plant model using the opitmized control inputs
sim(Plant);
save('ds1_plant_iter_1')

%save the states of the plant model at the end of the first time step
stance_ankle_p_out1 = stance_ankle_p_out(831);
stance_ankle_w_out1 = stance_ankle_w_out(831);
stance_hip_p_out1 = stance_hip_p_out(831);
stance_hip_w_out1 = stance_hip_w_out(831);
stance_knee_p_out1 = stance_knee_p_out(831);
stance_knee_w_out1 = stance_knee_w_out(831);
swing_ankle_p_out1 = swing_ankle_p_out(831);
swing_ankle_w_out1 = swing_ankle_w_out(831);
swing_knee_p_out1 = swing_knee_p_out(831);
swing_knee_w_out1 = swing_knee_w_out(831);
swing_hip_p_out1 = swing_hip_p_out(831);
swing_hip_w_out1 = swing_hip_w_out(831);
Planar_joint_x_p_out1 = Planar_joint_x_p_out(831);
Planar_joint_x_v_out1 = Planar_joint_x_v_out(831);
Planar_joint_y_p_out1 = Planar_joint_y_p_out(831);
Planar_joint_y_v_out1 = Planar_joint_y_v_out(831);
Planar_joint_z_p_out1 = Planar_joint_z_p_out(831);
Planar_joint_z_w_out1 = Planar_joint_z_w_out(831);

save('ds1_nextoptIC_iter_1','Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
save('ds1_controlinputhistory_iter_1','u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

toc

for i = 1:19
    tic 
    
    % Clear all the variables except for the counter i
    clearvars -except i directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc
    
    % Regress the current simulation time
    tnow = .0083*i;
    
    % Open human gait plant model and internal MPC model
    sys = 'ds1_inter';
    open_system(sys);
    
    % Set the designed variables for the MPC optimization
    filename = ['ds1_optimizedcontrolinput_iter_' num2str(i)];
    load(filename);
    
    % Set the maximum, minimum allowable values and the scales of the design variables
    p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagStanceKnee','lagSwingAnkle','lagSwingKnee'});
        p(1).Minimum = u(1,:)-15*ones(1,6);
        p(2).Minimum = u(2,:)-3*ones(1,6);
        p(3).Minimum = u(3,:)-1*ones(1,6);
        p(4).Minimum = u(4,:)-1.5*ones(1,6);
        p(1).Maximum = u(1,:)+15*ones(1,6);
        p(2).Maximum = u(2,:)+3*ones(1,6);
        p(3).Maximum = u(3,:)+1*ones(1,6);
        p(4).Maximum = u(4,:)+1.5*ones(1,6);
            
    for kk = 1:4
        laguerre_coefficient_no_count = 6;
        if kk == 1
            p_Min = -150;
            p_Max = 150;            
        elseif kk == 2
            p_Min = -30;
            p_Max = 30;
        elseif kk == 3
            p_Min = -8;
            p_Max = 8;
        else 
            p_Min = -15;
            p_Max = 15;
        end
        
        for kkk = 1:laguerre_coefficient_no_count
            if p(kk).Minimum(kkk) < p_Min
                p(kk).Minimum(kkk) = p_Min;
            end
            if p(kk).Maximum(kkk) > p_Max
                p(kk).Maximum(kkk) = p_Max;
            end
            if abs(p(kk).Maximum(kkk)) >= abs(p(kk).Minimum(kkk))
                p(kk).Scale(kkk) = abs(p(kk).Maximum(kkk));
            else
                p(kk).Scale(kkk) = abs(p(kk).Minimum(kkk));
            end
        end
    end
    
    p(1).Value = u(1,:);
    p(2).Value = u(2,:);
    p(3).Value = u(3,:);
    p(4).Value = u(4,:);
        
    %load saved states of the plant model at the end of the previous sample time
    filename = ['ds1_nextoptIC_iter_' num2str(i)];
    load(filename);
    
    % Define the initial condition for the internal MPC model for the next optimization step
    sdo.setValueInModel(sys,'beginTime',tnow);
    sdo.setValueInModel(sys,'p_stance_ankle_i',stance_ankle_p_out1);
    sdo.setValueInModel(sys,'w_stance_ankle_i',stance_ankle_w_out1);
    sdo.setValueInModel(sys,'p_stance_knee_i',stance_knee_p_out1);
    sdo.setValueInModel(sys,'w_stance_knee_i',stance_knee_w_out1);
    sdo.setValueInModel(sys,'p_stance_hip_i',stance_hip_p_out1);
    sdo.setValueInModel(sys,'w_stance_hip_i',stance_hip_w_out1);
    sdo.setValueInModel(sys,'p_swing_ankle_i',swing_ankle_p_out1);
    sdo.setValueInModel(sys,'w_swing_ankle_i',swing_ankle_w_out1);
    sdo.setValueInModel(sys,'p_swing_knee_i',swing_knee_p_out1);
    sdo.setValueInModel(sys,'w_swing_knee_i',swing_knee_w_out1);
    sdo.setValueInModel(sys,'p_swing_hip_i',swing_hip_p_out1);
    sdo.setValueInModel(sys,'w_swing_hip_i',swing_hip_w_out1);
    % sdo.setValueInModel(sys,'p_stance_foot_i',Planar_joint_z_p_out1);
    % sdo.setValueInModel(sys,'w_stance_foot_i',Planar_joint_z_w_out1);
    sdo.setValueInModel(sys,'Planar_joint_x_p_i',Planar_joint_x_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_x_v_i',Planar_joint_x_v_out1);
    sdo.setValueInModel(sys,'Planar_joint_y_p_i',Planar_joint_y_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_y_v_i',Planar_joint_y_v_out1);
    sdo.setValueInModel(sys,'Planar_joint_z_p_i',Planar_joint_z_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_z_w_i',Planar_joint_z_w_out1);
    
    save_system('ds1_inter');
    
    % Create a simulation tester
    simulator = sdo.SimulationTest(sys);

    % Define the optimization algorithm
    addpath(sim_file_dir);
    costfxn = str2func(['objfunDS_' objfun_vers]);
    evalDesign = @(p) costfxn(p,simulator);
    
    % Select options for the optimizer
    opt = sdo.OptimizeOptions;
        opt.MethodOptions.Algorithm = 'interior-point';
        opt.MethodOptions.UseParallel = 'always';
        % opt.MethodOptions.DiffMaxChange = 2;
        % opt.MethodOptions.TypicalX = u;
        opt.MethodOptions.FinDiffType = 'central';
        % opt.MethodOptions.TolFun = 0.1;
        % opt.MethodOptions.ScaleProblem = 'obj-and-constr';
        % opt.MethodOptions.AlwaysHonorConstraints = 'none';
        opt.MethodOptions.ObjectiveLimit = 5;
        opt.UseParallel = 'always';
        opt.OptimizedModel = sys;
    
    clear u j
    clear stance_ankle_p_out1 stance_ankle_w_out1 stance_hip_p_out1 stance_hip_w_out1 stance_knee_p_out1 stance_knee_w_out1 swing_ankle_p_out1 swing_ankle_w_out1 swing_hip_p_out1 swing_hip_w_out1 swing_knee_p_out1 swing_knee_w_out1 Planar_joint_x_p_out1 Planar_joint_x_v_out1
    clear Planar_joint_y_p_out1 Planar_joint_y_v_out1 Planar_joint_z_p_out1 Planar_joint_z_w_out1
    clear u_stance_ankle_data_end u_stance_hip_data_end u_stance_knee_data_end u_swing_ankle_data_end u_swing_hip_data_end u_swing_knee_data_end
    clear kk kkk laguerre_coefficient_no_count p1_Max p1_Min p_Max p_Min

    [pOpt,optInfo] = sdo.optimize(evalDesign,p,opt);
    
    filename = ['ds1_inter_iter_' num2str(i+1)];
    save(filename)
    
    for ii = 1:4
        u(ii,:) = pOpt(ii).Value;
    end
    
    filename = ['ds1_optimizedcontrolinput_iter_' num2str(i+1)];
    save(filename,'u');
    
    % clear required parameters to be able to run the plant model simulation
    clearvars -except i directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc
   
    % load double support plant
    Plant = 'ds1_plant';
    
    % load initial conditions for next optimization
    filename = ['ds1_optimizedcontrolinput_iter_' num2str(i+1)];
    load(filename);
    filename = ['ds1_controlinputhistory_iter_' num2str(i)];
    load(filename);
    
    % simulate model and save the end states to be used as the initial conditions for the next time step
    sdo.setValueInModel(Plant,'BeginTimePlantModel',i*.0082);
    sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
    sdo.setValueInModel(Plant,'u_stance_knee',u(2,:));
    sdo.setValueInModel(Plant,'u_swing_ankle',u(3,:));
    sdo.setValueInModel(Plant,'u_swing_knee',u(4,:));
    
    u_stance_ankle_data_timeseries = timeseries;
    u_stance_ankle_data_timeseries.Time = 0:.00001:i*.0082;
    u_stance_ankle_data_timeseries.Data = u_stance_ankle_data(1:(i*820+1));
    u_stance_knee_data_timeseries = timeseries;
    u_stance_knee_data_timeseries.Time = 0:.00001:i*.0082;
    u_stance_knee_data_timeseries.Data = u_stance_knee_data(1:(i*820+1));
    u_swing_ankle_data_timeseries = timeseries;
    u_swing_ankle_data_timeseries.Time = 0:.00001:i*.0082;
    u_swing_ankle_data_timeseries.Data = u_swing_ankle_data(1:(i*820+1));
    u_swing_knee_data_timeseries = timeseries;
    u_swing_knee_data_timeseries.Time = 0:.00001:i*.0082;
    u_swing_knee_data_timeseries.Data = u_swing_knee_data(1:(i*820+1));
        
    clear u
    clear u_stance_ankle_data u_stance_knee_data u_stance_hip_data u_swing_ankle_data u_swing_knee_data u_swing_hip_data
    
    sim(Plant);
    
    % check if toe-off happens within iteration, if so, exit loop
    if lag_toe_zgrf <= 0 & lead_toe_z <= 0 & length(lag_toe_zgrf) <= (820*(i+1)+1)
        break
    end
    
    % Save all the variable after one iteration simulation of plant model into a file
    filename = ['ds1_plant_iter_' num2str(i+1)];
    save(filename);
    
    stance_ankle_p_out1 = stance_ankle_p_out((i+1)*820+1);
    stance_ankle_w_out1 = stance_ankle_w_out((i+1)*820+1);
    stance_hip_p_out1 = stance_hip_p_out((i+1)*820+1);
    stance_hip_w_out1 = stance_hip_w_out((i+1)*820+1);
    stance_knee_p_out1 = stance_knee_p_out((i+1)*820+1);
    stance_knee_w_out1 = stance_knee_w_out((i+1)*820+1);
    swing_ankle_p_out1 = swing_ankle_p_out((i+1)*820+1);
    swing_ankle_w_out1 = swing_ankle_w_out((i+1)*820+1);
    swing_knee_p_out1 = swing_knee_p_out((i+1)*820+1);
    swing_knee_w_out1 = swing_knee_w_out((i+1)*820+1);
    swing_hip_p_out1 = swing_hip_p_out((i+1)*820+1);
    swing_hip_w_out1 = swing_hip_w_out((i+1)*820+1);
    Planar_joint_x_p_out1 = Planar_joint_x_p_out((i+1)*820+1);
    Planar_joint_x_v_out1 = Planar_joint_x_v_out((i+1)*820+1);
    Planar_joint_y_p_out1 = Planar_joint_y_p_out((i+1)*820+1);
    Planar_joint_y_v_out1 = Planar_joint_y_v_out((i+1)*820+1);
    Planar_joint_z_p_out1 = Planar_joint_z_p_out((i+1)*820+1);
    Planar_joint_z_w_out1 = Planar_joint_z_w_out((i+1)*820+1);
    
    filename = ['ds1_nextoptIC_iter_' num2str(i+1)];
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
        
    filename = ['ds1_controlinputhistory_iter_' num2str(i+1)];
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    toc

end

% Save all the variable at toe off (or end of 19th iteration) of plant model into a file
filename = 'ds1_plant_end';
    ic.p(1) = stance_ankle_p_out(end);
    ic.w(1) = stance_ankle_w_out(end);
    ic.p(2) = stance_knee_p_out(end);
    ic.w(1) = stance_knee_w_out(end);
    ic.p(3) = stance_hip_p_out(end);
    ic.w(3) = stance_hip_w_out(end);
    ic.p(4) = swing_ankle_p_out(end);
    ic.w(4) = swing_ankle_w_out(end);
    ic.p(5) = swing_knee_p_out(end);
    ic.w(5) = swing_knee_w_out(end);
    ic.p(6) = swing_hip_p_out(end);
    ic.w(6) = swing_hip_w_out(end);
    save(filename, ic);

filename = 'ds1_nextoptIC_end';
save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

filename = 'ds1_controlinputhistory_end';
save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

cd(subdirectory)