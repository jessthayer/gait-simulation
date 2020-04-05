%% SS 1
global directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc...
    complete_ds1

mkdir('SS1') %create a folder for gait period
subdirectory = cd('SS1'); %open new folder

%% begin single support 1
delete(gcp('nocreate'))

diary on %record the history of the command window
diary SSCommandWindowHistory.txt

tic %start a timer to record time of first iteration of optimization

%Start the parallel computing
parpool

% Open human gait plant model and internal MPC models
open_system(ss_inter_loc);
save_system(ss_inter_loc, 'ss1_inter');
open_system(ss_plant_loc);
save_system(ss_plant_loc, 'ss1_plant');

% Current time is set to be 0
tnow = 0;

% Open internal model
sys = 'ss1_inter';
open_system(sys);

%set internal model initial conditions
load([sim_file_dir '\' trialName '\DS1\ds1_plant_end'])
    sdo.setValueInModel(sys,'p_stance_ankle_i',ic.p(4));
    sdo.setValueInModel(sys,'w_stance_ankle_i',ic.w(4));
    sdo.setValueInModel(sys,'p_stance_knee_i',ic.p(5));
    sdo.setValueInModel(sys,'w_stance_knee_i',ic.w(5));
    sdo.setValueInModel(sys,'p_stance_hip_i',ic.p(6));
    sdo.setValueInModel(sys,'w_stance_hip_i',ic.w(6));
    sdo.setValueInModel(sys,'p_swing_ankle_i',ic.p(1));
    sdo.setValueInModel(sys,'w_swing_ankle_i',ic.w(1));
    sdo.setValueInModel(sys,'p_swing_knee_i',ic.p(2));
    sdo.setValueInModel(sys,'w_swing_knee_i',ic.w(2));
    sdo.setValueInModel(sys,'p_swing_hip_i',ic.p(3));
    sdo.setValueInModel(sys,'w_swing_hip_i',ic.w(3));
save_system('ss1_inter');

% Get the design variables for the MPC optimization
p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagSwingAnkle','lagSwingKnee',...
    'lagSwingHip','w_stance_ankle_i','w_swing_knee_i','w_swing_hip_i'});

% Set the maximum and minimum allowable values of the design variables
p(1).Minimum = [-100 -300 -100 -30 -10 -100];
p(2).Minimum = -20*ones(1,6);
p(3).Minimum = -40*ones(1,6);
p(4).Minimum = -55*ones(1,6);
p(1).Maximum = [100 300 100 30 10 100];
p(2).Maximum = 20*ones(1,6);
p(3).Maximum = 40*ones(1,6);
p(4).Maximum = 55*ones(1,6);
temp1 = p(5).Value*0.95;
temp2 = p(5).Value*1.05;
if temp1 < temp2
    p(5).Minimum = temp1;
    p(5).Maximum = temp2;
else
    p(5).Minimum = temp2;
    p(5).Maximum = temp1;
end
temp1 = p(6).Value*0.8;
temp2 = p(6).Value*1.2;
if temp1 < temp2
    p(6).Minimum = temp1;
    p(6).Maximum = temp2;
else
    p(6).Minimum = temp2;
    p(6).Maximum = temp1;
end
temp1 = p(7).Value*0.8;
temp2 = p(7).Value*1.2;
if temp1 < temp2
    p(7).Minimum = temp1;
    p(7).Maximum = temp2;
else
    p(7).Minimum = temp2;
    p(7).Maximum = temp1;
end

% Set the scale of the design variables
p(1).Scale = [100 300 100 30 10 100];
p(2).Scale = 20*ones(1,6);
p(3).Scale = 40*ones(1,6);
p(4).Scale = 55*ones(1,6);
p(5).Scale = abs(p(5).Value);
p(6).Scale = abs(p(6).Value);
p(7).Scale = abs(p(7).Value);

% Create a simulation tester
simulator = sdo.SimulationTest(sys);

% Define the optimization algorithm
addpath(sim_file_dir);
costfxn = str2func(['objfunSS_' objfun_vers]);
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
opt.MethodOptions.MaxIter = 10;
opt.MethodOptions.TolX = 0.02;
opt.UseParallel = 'always';
opt.OptimizedModel = sys;

% Start the optimization
[pOpt,optInfo] = sdo.optimize(evalDesign,p,opt);

save('ss1_inter_iter_1');

for i = 1:7
    u(i,:) = pOpt(i).Value;
end

save('ss1_optimizedcontrolinput_iter_1','u');

%set plant initial conditions
load([sim_file_dir '\' trialName '\DS1\ds1_plant_end'])
Plant = 'ss1_plant';
sdo.setValueInModel(Plant,'p_stance_ankle',ic.p(4));
sdo.setValueInModel(Plant,'p_stance_knee',ic.p(5));
sdo.setValueInModel(Plant,'p_stance_hip',ic.p(6));
sdo.setValueInModel(Plant,'p_swing_ankle',ic.p(1));
sdo.setValueInModel(Plant,'p_swing_knee',ic.p(2));
sdo.setValueInModel(Plant,'p_swing_hip',ic.p(3));
sdo.setValueInModel(Plant,'w_stance_ankle',ic.w(4));
sdo.setValueInModel(Plant,'w_stance_knee',ic.w(5));
sdo.setValueInModel(Plant,'w_stance_hip',ic.w(6));
sdo.setValueInModel(Plant,'w_swing_ankle',ic.w(1));
sdo.setValueInModel(Plant,'w_swing_knee',ic.w(2));
sdo.setValueInModel(Plant,'w_swing_hip',ic.w(3));

%send optimized control input to plant
sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
sdo.setValueInModel(Plant,'u_swing_ankle',u(2,:));
sdo.setValueInModel(Plant,'u_swing_knee',u(3,:));
sdo.setValueInModel(Plant,'u_swing_hip',u(4,:));
sdo.setValueInModel(Plant,'w_stance_ankle',u(5));
sdo.setValueInModel(Plant,'w_swing_knee',u(6));
sdo.setValueInModel(Plant,'w_swing_hip',u(7));

u_stance_ankle_data_timeseries = timeseries;
u_stance_ankle_data_timeseries.Time = 0;
u_stance_ankle_data_timeseries.Data = 0;
u_swing_ankle_data_timeseries = timeseries;
u_swing_ankle_data_timeseries.Time = 0;
u_swing_ankle_data_timeseries.Data = 0;
u_swing_knee_data_timeseries = timeseries;
u_swing_knee_data_timeseries.Time = 0;
u_swing_knee_data_timeseries.Data = 0;
u_swing_hip_data_timeseries = timeseries;
u_swing_hip_data_timeseries.Time = 0;
u_swing_hip_data_timeseries.Data = 0;

clear u;

%simulate plant model using the opitmized control inputs
sim(Plant);
save('ss1_plant_iter_1');

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

save('ss1_nextoptIC_iter_1','Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
save('ss1_controlinputhistory_iter_1','u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

toc

for i = 1:47
    tic
    
    % Clear all the variables except for the counter i
    clearvars -except i directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc...
    complete_ds1
    
    % Regress the current simulation time
    tnow = .0083*i;
    
    % Open human gait plant model and internal MPC model
    sys = 'ss1_inter';
    open_system(sys);
    
    % Set the designed variables for the MPC optimization
    filename = ['ss1_optimizedcontrolinput_iter_' num2str(i)];
    load(filename); 
    
    % Set the maximum, minimum allowable values and the scales of the 
    % design variables
    p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagSwingAnkle','lagSwingKnee','lagSwingHip'});
        p(1).Minimum = [u(1,1)-10 u(1,2)-30 u(1,3)-10 u(1,4)-3 u(1,5)-1 u(1,6)-1];
        p(2).Minimum = u(2,:)-2*ones(1,6);
        p(3).Minimum = u(3,:)-4*ones(1,6);
        p(4).Minimum = u(4,:)-5*ones(1,6);
        p(1).Maximum = [u(1,1)+10 u(1,2)+30 u(1,3)+10 u(1,4)+3 u(1,5)+1 u(1,6)+1];
        p(2).Maximum = u(2,:)+2*ones(1,6);
        p(3).Maximum = u(3,:)+4*ones(1,6);
        p(4).Maximum = u(4,:)+5*ones(1,6);
    
    p1_Min = [-100 -300 -100 -30 -10 -100];
    p1_Max = [100 300 100 30 10 100];
    
    for kk = 1:5
        if p(1).Minimum(kk) < p1_Min(kk)
            p(1).Minimum(kk) = p1_Min(kk);
        end
        if p(1).Maximum(kk) > p1_Max(kk)
            p(1).Maximum(kk) = p1_Max(kk);
        end
        if abs(p(1).Maximum(kk)) >= abs(p(1).Minimum(kk))
            p(1).Scale(kk) = abs(p(1).Maximum(kk));
        else
            p(1).Scale(kk) = abs(p(1).Minimum(kk));
        end
    end
    
        for kk = 2:4
            laguerre_coefficient_no_count = 6;
            if kk == 2
                p_Min = -20;
                p_Max = 20;
            elseif kk == 3
                p_Min = -40;
                p_Max = 40;
            else 
                p_Min = -55;
                p_Max = 55;
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
    filename = ['ss1_nextoptIC_iter_' num2str(i)];
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
    sdo.setValueInModel(sys,'x_planar_p_i',Planar_joint_x_p_out1);
    sdo.setValueInModel(sys,'x_planar_v_i',Planar_joint_x_v_out1);
    sdo.setValueInModel(sys,'y_planar_p_i',Planar_joint_y_p_out1);
    sdo.setValueInModel(sys,'y_planar_v_i',Planar_joint_y_v_out1);
    sdo.setValueInModel(sys,'z_planar_p_i',Planar_joint_z_p_out1);
    sdo.setValueInModel(sys,'z_planar_v_i',Planar_joint_z_w_out1);
    
    save_system('ss1_inter');

    % Create a simulation tester
    simulator = sdo.SimulationTest(sys);
    
    % Define the optimization algorithm
    addpath(sim_file_dir);
    costfxn = str2func(['objfunSS_' objfun_vers]);
    evalDesign = @(p) costfxn(p,simulator);
    
    % Select options for the optimizer
    opt = sdo.OptimizeOptions;
        opt.MethodOptions.Algorithm = 'interior-point';
        opt.MethodOptions.UseParallel = 'always';
        % opt.MethodOptions.DiffMaxChange = 2;
        % opt.MethodOptions.TypicalX = u;
        opt.MethodOptions.FinDiffType = 'central';
        opt.MethodOptions.TolFun = 0.1;
        % opt.MethodOptions.ScaleProblem = 'obj-and-constr';
        % opt.MethodOptions.AlwaysHonorConstraints = 'none';
        opt.MethodOptions.ObjectiveLimit = 5;
        opt.MethodOptions.MaxIter = 20;
        opt.MethodOptions.TolX = 0.02;
        opt.UseParallel = 'always';
        opt.OptimizedModel = sys;
    
    clear u j
    clear stance_ankle_p_out1 stance_ankle_w_out1 stance_hip_p_out1 stance_hip_w_out1 stance_knee_p_out1 stance_knee_w_out1 swing_ankle_p_out1 swing_ankle_w_out1 swing_hip_p_out1 swing_hip_w_out1 swing_knee_p_out1 swing_knee_w_out1 Planar_joint_x_p_out1 Planar_joint_x_v_out1
    clear Planar_joint_y_p_out1 Planar_joint_y_v_out1 Planar_joint_z_p_out1 Planar_joint_z_w_out1
    clear u_stance_ankle_data_end u_stance_hip_data_end u_stance_knee_data_end u_swing_ankle_data_end u_swing_hip_data_end u_swing_knee_data_end
    clear kk kkk laguerre_coefficient_no_count p1_Max p1_Min p_Max p_Min
  
    [pOpt,optInfo] = sdo.optimize(evalDesign,p,opt);
    
    filename = ['ss1_inter_iter_' num2str(i+1)];
    save(filename)
    
    for ii = 1:4
        u(ii,:) = pOpt(ii).Value;
    end
    
    filename = ['ss1_optimizedcontrolinput_iter_' num2str(i+1)];
    save(filename,'u');
    
    % Load the reuqired parameter to be able to run the plant model simulation
    clearvars -except i directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc...
    complete_ds1
    
    % load swing plant model
    Plant = 'ss1_plant';
    
    % load IC_for_next_opt
    filename = ['ss1_optimizedcontrolinput_iter_' num2str(i+1)];
    load(filename);
    filename = ['ss1_controlinputhistory_iter_' num2str(i)];
    load(filename);
    
    % Simulate model and save the end states to be used as the initial conditions for the next time step
    sdo.setValueInModel(Plant,'BeginTimePlantModel',i*.0083);
    sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
    sdo.setValueInModel(Plant,'u_swing_ankle',u(2,:));
    sdo.setValueInModel(Plant,'u_swing_knee',u(3,:));
    sdo.setValueInModel(Plant,'u_swing_hip',u(4,:));
    
    u_stance_ankle_data_timeseries = timeseries;
    u_stance_ankle_data_timeseries.Time = 0:.00001:i*.0083;
    u_stance_ankle_data_timeseries.Data = u_stance_ankle_data(1:(i*830+1));
    u_swing_ankle_data_timeseries = timeseries;
    u_swing_ankle_data_timeseries.Time = 0:.00001:i*.0083;
    u_swing_ankle_data_timeseries.Data = u_swing_ankle_data(1:(i*830+1));
    u_swing_knee_data_timeseries = timeseries;
    u_swing_knee_data_timeseries.Time = 0:.00001:i*.0083;
    u_swing_knee_data_timeseries.Data = u_swing_knee_data(1:(i*830+1));
    u_swing_hip_data_timeseries = timeseries;
    u_swing_hip_data_timeseries.Time = 0:.00001:i*.0083;
    u_swing_hip_data_timeseries.Data = u_swing_hip_data(1:(i*830+1));
    
    clear u
    clear u_stance_ankle_data u_stance_knee_data u_stance_hip_data u_swing_ankle_data u_swing_knee_data u_swing_hip_data
        
    sim(Plant);
    
    % check if heel contact happens within iteration, if so, exit loop
    if swing_heel_ground_contact(end) == 1 && length(swing_heel_ground_contact) <= (830*(i+1)+1)
        break
    end
    
    % Save all the variables after one iteration simulation of plant model into a file
    filename = ['ss1_plant_iter_' num2str(i+1)];
    save(filename);
    
    stance_ankle_p_out1 = stance_ankle_p_out((i+1)*830+1);
    stance_ankle_w_out1 = stance_ankle_w_out((i+1)*830+1);
    stance_hip_p_out1 = stance_hip_p_out((i+1)*830+1);
    stance_hip_w_out1 = stance_hip_w_out((i+1)*830+1);
    stance_knee_p_out1 = stance_knee_p_out((i+1)*830+1);
    stance_knee_w_out1 = stance_knee_w_out((i+1)*830+1);
    swing_ankle_p_out1 = swing_ankle_p_out((i+1)*830+1);
    swing_ankle_w_out1 = swing_ankle_w_out((i+1)*830+1);
    swing_knee_p_out1 = swing_knee_p_out((i+1)*830+1);
    swing_knee_w_out1 = swing_knee_w_out((i+1)*830+1);
    swing_hip_p_out1 = swing_hip_p_out((i+1)*830+1);
    swing_hip_w_out1 = swing_hip_w_out((i+1)*830+1);
    Planar_joint_x_p_out1 = Planar_joint_x_p_out((i+1)*830+1);
    Planar_joint_x_v_out1 = Planar_joint_x_v_out((i+1)*830+1);
    Planar_joint_y_p_out1 = Planar_joint_y_p_out((i+1)*830+1);
    Planar_joint_y_v_out1 = Planar_joint_y_v_out((i+1)*830+1);
    Planar_joint_z_p_out1 = Planar_joint_z_p_out((i+1)*830+1);
    Planar_joint_z_w_out1 = Planar_joint_z_w_out((i+1)*830+1);
    
    filename = ['ss1_nextoptIC_iter_' num2str(i+1)];
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

    filename = ['ss1_controlinputhistory_iter_' num2str(i+1)];
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    toc
    
end


% Save all the variable at heel strike (or end of 50th iteration) of plant model into a file
    filename = 'ss1_plant_end';
    save(filename);
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
        Planar_joint_x_p_out1 = Planar_joint_x_p_out(end);
        Planar_joint_x_v_out1 = Planar_joint_x_v_out(end);
        Planar_joint_y_p_out1 = Planar_joint_y_p_out(end);
        Planar_joint_y_v_out1 = Planar_joint_y_v_out(end);
        Planar_joint_z_p_out1 = Planar_joint_z_p_out(end);
        Planar_joint_z_w_out1 = Planar_joint_z_w_out(end);
        swing_heel_ground_contact1 = swing_heel_ground_contact(end);
        Toe_L_x_ss = Toe_L_x;
        Toe_L_y_ss = Toe_L_y;
        Toe_L_z_ss = Toe_L_z;
        Heel_L_z_ss = Heel_L_z;
        Toe_R_x_ss = Toe_L_x;
        Toe_R_y_ss = Toe_L_y;
        Toe_R_z_ss = Toe_L_z;
        Heel_R_z_ss = Heel_L_z;  
    
    filename = 'ss_1_nextoptIC_end';
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

    filename = 'ss_1_controlinputhistory_end';
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    filename = 'ss_1_feetlocation_end';
    save(filename,'Toe_L_x_ss','Toe_L_y_ss','Toe_L_z_ss','Heel_L_z_ss','Toe_R_x_ss','Toe_R_y_ss','Toe_R_z_ss','Heel_R_z_ss');
    
    cd(subdirectory)