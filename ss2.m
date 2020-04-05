% minimizes COM energy changes

%% define IC and anthropomorphic measurements
num = 14;
sysID = ['Trial' 14]; %set this to match desired sysID from IC Script

%% begin swing 1
delete(gcp('nocreate'))

diary on %record the history of the command window
diary SSCommandWindowHistory.txt

tic %start a timer to record time of first iteration of optimization

%Start the parallel computing
parpool

% Current time is set to be 0
tnow = 0;

% Open human gait plant model and internal MPC models
% save as first single support phase
open_system('ss_inter_v2_i'); 
save_system('ss_inter_v2_i','ss_inter_v2');
open_system('ss_plant_v2_i');
save_system('ss_plant_v2_i','ss_plant_v2');

sys = 'ss_inter_v2';
open_system(sys);

%set internal model initial conditions
load('C:\Users\8842thayerj\OneDrive - Marquette University\Research\Simulations\Initial Condition Stability\Initial Conditions\ic_trial014')
sdo.setValueInModel(sys,'p_stance_ankle_i',ic.p(1));
sdo.setValueInModel(sys,'w_stance_ankle_i',ic.w(1));
sdo.setValueInModel(sys,'p_stance_knee_i',ic.p(2));
sdo.setValueInModel(sys,'w_stance_knee_i',ic.w(2));
sdo.setValueInModel(sys,'p_stance_hip_i',ic.p(3));
sdo.setValueInModel(sys,'w_stance_hip_i',ic.w(3));
sdo.setValueInModel(sys,'p_swing_ankle_i',ic.p(4));
sdo.setValueInModel(sys,'w_swing_ankle_i',ic.w(4));
sdo.setValueInModel(sys,'p_swing_knee_i',ic.p(5));
sdo.setValueInModel(sys,'w_swing_knee_i',ic.w(5));
sdo.setValueInModel(sys,'p_swing_hip_i',ic.p(6));
sdo.setValueInModel(sys,'w_swing_hip_i',ic.w(6));

% Set the designed variables for the MPC optimization
p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagSwingAnkle','lagSwingKnee',...
    'lagSwingHip','w_stance_ankle_i','w_swing_knee_i','w_swing_hip_i'});

save_system('ss_inter_v2');

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
evalDesign = @(p) objfunSS_v2(p,simulator);

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
[pOpt,optInfo] = sdo.optimize(evalDesign,p,opt); % fmincon (the default optimization method) to solve a design optimization problem

save('ss_inter_v2_iter_1');

for i = 1:7
    u(i,:) = pOpt(i).Value;
end

save('ss_optimizedcontrolinput_iter_1','u');

% Load the human gait anthropometric data and the optimized control input
Plant = 'ss_plant_v2';

%set plant initial conditions
load(['C:\Users\8842thayerj\OneDrive - Marquette University\Research\Simulations\Initial Condition Stability\Initial Conditions\ic_trial0',num2str(num)])
sdo.setValueInModel(Plant,'p_stance_ankle',ic.p(1));
sdo.setValueInModel(Plant,'p_stance_knee',ic.p(2));
sdo.setValueInModel(Plant,'p_stance_hip',ic.p(3));
sdo.setValueInModel(Plant,'p_swing_ankle',ic.p(4));
sdo.setValueInModel(Plant,'p_swing_knee',ic.p(5));
sdo.setValueInModel(Plant,'p_swing_hip',ic.p(6));
sdo.setValueInModel(Plant,'w_stance_ankle',ic.w(1));
sdo.setValueInModel(Plant,'w_stance_knee',ic.w(2));
sdo.setValueInModel(Plant,'w_stance_hip',ic.w(3));
sdo.setValueInModel(Plant,'w_swing_ankle',ic.w(4));
sdo.setValueInModel(Plant,'w_swing_knee',ic.w(5));
sdo.setValueInModel(Plant,'w_swing_hip',ic.w(6));

%load control input
load ss_optimizedcontrolinput_iter_1

% Send the optimized values of the design variables to the plant
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

% Simulate plant model using the opitmized control inputs
sim(Plant);
save('ss_plant_v2_iter_1');

% Save the states of the plant model at the end of the first time step.
% They will be used as the initial state of the plant model for the next
% time step
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

save('ss_nextoptIC_iter_1','Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
save('ss_controlinputhistory_iter_1','u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

toc

for i = 1:49
    tic
    
    % Clear all the variables except for the counter i to perform
    % optimization for internal model again
    clearvars -except i
    
    % Regress the current simulation time
    tnow = .0083*i;
    
    % Open human gait plant model and internal MPC model
    sys = 'ss_inter_v2';
    open_system(sys);
    
    % Set the designed variables for the MPC optimization
    filename = ['ss_optimizedcontrolinput_iter_' num2str(i)];
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
        if kk == 2
            laguerre_coefficient_no_count = 6;
            p_Min = -20;
            p_Max = 20;
        elseif kk == 3
            laguerre_coefficient_no_count = 6;
            p_Min = -40;
            p_Max = 40;
        else 
            laguerre_coefficient_no_count = 6;
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
    
    % Load the saved states of the plant model at the end of the previous sample time
    filename = ['ss_nextoptIC_iter_' num2str(i)];
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
    
    save_system('ss_inter_v2');

    % Create a simulation tester
    simulator = sdo.SimulationTest(sys);
    
    % Define the optimization algorithm
    evalDesign = @(p) objfunSS_v2(p,simulator);
    
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
    
    filename = ['ss_inter_v2_iter_' num2str(i+1)];
    save(filename)
    
    for i = 1:4
        u(i,:) = pOpt(i).Value;
    end
    
    save('ss_optimizedcontrolinput_iter_1','u');
    
    % Load the reuqired parameter to be able to run the plant model simulation
    clearvars -except i
    
    % load swing plant model
    Plant = 'ss_plant_v2_1';
    
    % load IC_for_next_opt
    filename = ['ss_optimizedcontrolinput_iter_' num2str(i+1)];
    load(filename);
    filename = ['ss_controlinputhistory_iter_' num2str(i)];
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
    if swing_heel_ground_contact(end) == 1 && length(swing_heel_ground_contact) <= (831*(i+1)+1)
        break
    end
    
    % Save all the variables after one iteration simulation of plant model into a file
    filename = ['ss_plant_v2_iter_' num2str(i+1)];
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
    
    filename = ['ss_nextoptIC_iter_' num2str(i+1)];
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

    filename = ['ss_controlinputhistory_iter_' num2str(i+1)];
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    toc
    
end


% Save all the variable at heel strike (or end of 50th iteration) of plant model into a file
    filename = 'ss_plant_v2_end';
    save(filename);
    
    stance_ankle_p_out1 = stance_ankle_p_out(end);
    stance_ankle_w_out1 = stance_ankle_w_out(end);
    stance_hip_p_out1 = stance_hip_p_out(end);
    stance_hip_w_out1 = stance_hip_w_out(end);
    stance_knee_p_out1 = stance_knee_p_out(end);
    stance_knee_w_out1 = stance_knee_w_out(end);
    swing_ankle_p_out1 = swing_ankle_p_out(end);
    swing_ankle_w_out1 = swing_ankle_w_out(end);
    swing_knee_p_out1 = swing_knee_p_out(end);
    swing_knee_w_out1 = swing_knee_w_out(end);
    swing_hip_p_out1 = swing_hip_p_out(end);
    swing_hip_w_out1 = swing_hip_w_out(end);
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
    
    
    
    filename = 'ss_nextoptIC_end';
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

    filename = 'ss_controlinputhistory_end';
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    filename = 'ss_feetlocation_end';
    save(filename,'Toe_L_x_ss','Toe_L_y_ss','Toe_L_z_ss','Heel_L_z_ss','Toe_R_x_ss','Toe_R_y_ss','Toe_R_z_ss','Heel_R_z_ss');
    

%% begin stance 1

delete(gcp('nocreate'))

% Record the history of the command window
diary on
diary DSCommandWindowHistory.txt

% Start a timer to record how long the first iteration of optimization takes
tic

% Start the parallel computing
parpool

% Open human gait plant model and internal MPC model
open_system('ds_inter_i');
save_system('ds_inter_v2_i','ds_inter_v2');
open_system('ds_plant_v2_i');
save_system('ds_plant_v2_i','ds_plant_v2');

% Load the saved states of the plant model at the end ss phase
filename = 'ss_nextoptIC_end';
load(filename);

% Overwrite time from single support for the start of double support
tnow = 0;

% Load initial conditions from swing plant to stance plant
plant = 'ds_plant_v2_1';
sdo.setValueInModel(plant,'BeginTimePlantModel',tnow);
sdo.setValueInModel(plant,'p_stance_ankle',stance_ankle_p_out1(end));
sdo.setValueInModel(plant,'w_stance_ankle',stance_ankle_w_out1(end));
sdo.setValueInModel(plant,'p_stance_knee',stance_knee_p_out1(end));
sdo.setValueInModel(plant,'w_stance_knee',stance_knee_w_out1(end));
sdo.setValueInModel(plant,'p_stance_hip',stance_hip_p_out1(end));
sdo.setValueInModel(plant,'w_stance_hip',stance_hip_w_out1(end));
sdo.setValueInModel(plant,'p_swing_ankle',swing_ankle_p_out1(end));
sdo.setValueInModel(plant,'w_swing_ankle',swing_ankle_w_out1(end));
sdo.setValueInModel(plant,'p_swing_knee',swing_knee_p_out1(end));
sdo.setValueInModel(plant,'w_swing_knee',swing_knee_w_out1(end));
sdo.setValueInModel(plant,'p_swing_hip',swing_hip_p_out1(end));
sdo.setValueInModel(plant,'w_swing_hip',swing_hip_w_out1(end));
sdo.setValueInModel(plant,'x_planar_p',Planar_joint_x_p_out1);
sdo.setValueInModel(plant,'x_planar_v',Planar_joint_x_v_out1);
sdo.setValueInModel(plant,'y_planar_p',Planar_joint_y_p_out1);
sdo.setValueInModel(plant,'y_planar_v',Planar_joint_y_v_out1);
sdo.setValueInModel(plant,'z_planar_p',Planar_joint_z_p_out1);
sdo.setValueInModel(plant,'z_planar_w',Planar_joint_z_w_out1);

save_system('ds_plant_v2_1');

% Open internal model
sys = 'ds_inter_v2';
open_system(sys);

% Get the designed variables for the MPC optimization
p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagStanceKnee','lagSwingAnkle','lagSwingKnee','w_stance_ankle_i', 'w_stance_knee_i','w_stance_hip_i','w_swing_knee_i','w_swing_hip_i'});
    % p(1) - laguerre polynomial stance ankle/left ankle (6 coeff)
    % p(2) - laguerre polynomial stance knee/left knee (6 coeff)
    % p(3) - laguerre polynomial swing ankle/right ankle (6 coeff)
    % p(4) - laguerre polynomial swing knee/right knee (6 coeff)
    % p(5) - angular velocity stance ankle/left ankle
    % p(6) - angular velocity stance knee/left knee
    % p(7) - angular velocity stance hip/left hip
    % p(8) - angular velocity swing knee/right knee
    % p(9) - angular velocity swing hip/right hip
  
% Set initial conditions in from swing plant to stance internal model
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
sdo.setValueInModel(sys,'Planar_joint_x_p_i',Planar_joint_x_p_out1);
sdo.setValueInModel(sys,'Planar_joint_x_v_i',Planar_joint_x_v_out1);
sdo.setValueInModel(sys,'Planar_joint_y_p_i',Planar_joint_y_p_out1);
sdo.setValueInModel(sys,'Planar_joint_y_v_i',Planar_joint_y_v_out1);
sdo.setValueInModel(sys,'Planar_joint_z_p_i',Planar_joint_z_p_out1);
sdo.setValueInModel(sys,'Planar_joint_z_w_i',Planar_joint_z_w_out1);

save_system('ds_inter_v2');

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
evalDesign = @(p) objfunDS(p,simulator);

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
save('ds_inter_v2_iter_1');

u(1,:) = pOpt(1).Value;
u(2,:) = pOpt(2).Value;
u(3,:) = pOpt(3).Value;
u(4,:) = pOpt(4).Value;
% we may not have to optimize these here since we put them in with ss ic
u5 = pOpt(5).Value;
u6 = pOpt(6).Value;
u7 = pOpt(7).Value;
u8 = pOpt(8).Value;
u9 = pOpt(9).Value;

save('ds_optimizedcontrolinput_iter_1','u(1,:)','u(2,:)','u(3,:)','u(4,:)','u5','u6','u7','u8','u9');

clear all

% Load the human gait anthropometric data and the optimized control input
Plant = 'ds_plant_v2_1';
load ds_optimizedcontrolinput_iter_1

% Send the optimized values of the design variables to the human gait plant
% model
sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
sdo.setValueInModel(Plant,'u_stance_knee',u(2,:));
sdo.setValueInModel(Plant,'u_swing_ankle',u(3,:));
sdo.setValueInModel(Plant,'u_swing_knee',u(4,:));
sdo.setValueInModel(Plant,'w_stance_ankle',u5);
sdo.setValueInModel(Plant,'w_stance_knee',u6);
sdo.setValueInModel(Plant,'w_stance_hip',u7);
sdo.setValueInModel(Plant,'w_swing_knee',u8);
sdo.setValueInModel(Plant,'w_swing_hip',u9);

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

clear u(1,:) u(2,:) u(3,:) u(4,:) u5 u6 u7 u8 u9

% Simulate plant model using the opitmized control inputs
sim(Plant);
save('ds_plant_v2_1_iter_1')

% Save the states of the plant model at the end of the first time step.
% They will be used as the initial state of the plant model for the next time step
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

save('ds_nextoptIC_iter_1','Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
save('ds_controlinputhistory_iter_1','u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

toc

for i = 1:19
    tic 
    
    % Clear all the variables except for the counter i to perform
    % optimization for internal model again
    clearvars -except i
    
    % Regress the current simulation time
    tnow = .0083*i;
    
    % Open human gait plant model and internal MPC model
    sys = 'ds_inter_v2';
    open_system(sys);
    
    % Set the designed variables for the MPC optimization
    filename = ['ds_optimizedcontrolinput_iter_' num2str(i)];
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
        if kk == 1
            laguerre_coefficient_no_count = 6;
            p_Min = -150;
            p_Max = 150;            
        elseif kk == 2
            laguerre_coefficient_no_count = 6;
            p_Min = -30;
            p_Max = 30;
        elseif kk == 3
            laguerre_coefficient_no_count = 6;
            p_Min = -8;
            p_Max = 8;
        else 
            laguerre_coefficient_no_count = 6;
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
        
    % Load the saved states of the plant model at the end of the previous
    % sample time
    filename = ['ds_nextoptIC_iter_' num2str(i)];
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
    save_system('ds_inter_v2');
    
    % Create a simulation tester
    simulator = sdo.SimulationTest(sys);

    % Define the optimization algorithm
    evalDesign = @(p) objfunDS(p,simulator);
    
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
    
    clear u(1,:) u(2,:) u(3,:) u(4,:) u5 u6 u7 u8 u9 j
    clear stance_ankle_p_out1 stance_ankle_w_out1 stance_hip_p_out1 stance_hip_w_out1 stance_knee_p_out1 stance_knee_w_out1 swing_ankle_p_out1 swing_ankle_w_out1 swing_hip_p_out1 swing_hip_w_out1 swing_knee_p_out1 swing_knee_w_out1 Planar_joint_x_p_out1 Planar_joint_x_v_out1
    clear Planar_joint_y_p_out1 Planar_joint_y_v_out1 Planar_joint_z_p_out1 Planar_joint_z_w_out1
    clear u_stance_ankle_data_end u_stance_hip_data_end u_stance_knee_data_end u_swing_ankle_data_end u_swing_hip_data_end u_swing_knee_data_end
    clear kk kkk laguerre_coefficient_no_count p1_Max p1_Min p_Max p_Min

    [pOpt,optInfo] = sdo.optimize(evalDesign,p,opt);
    
    filename = ['ds_inter_v2_iter_' num2str(i+1)];
    save(filename)
    
    u(1,:) = pOpt(1).Value;
    u(2,:) = pOpt(2).Value;
    u(3,:) = pOpt(3).Value;
    u(4,:) = pOpt(4).Value;
    filename = ['ds_optimizedcontrolinput_iter_' num2str(i+1)];
    save(filename,'u(1,:)','u(2,:)','u(3,:)','u(4,:)');
    
    % clear required parameters to be able to run the plant model simulation
    clearvars -except i
   
    % load double support plant
    Plant = 'ds_plant_v2_1';
    
    % load initial conditions for next optimization
    filename = ['ds_optimizedcontrolinput_iter_' num2str(i+1)];
    load(filename);
    filename = ['ds_controlinputhistory_iter_' num2str(i)];
    load(filename);
    
    % simulate model and save the end states to be used as the initial conditions for the next time step
    sdo.setValueInModel(Plant,'BeginTimePlantModel',i*.0083);
    sdo.setValueInModel(Plant,'u_stance_ankle',u(1,:));
    sdo.setValueInModel(Plant,'u_stance_knee',u(2,:));
    sdo.setValueInModel(Plant,'u_swing_ankle',u(3,:));
    sdo.setValueInModel(Plant,'u_swing_knee',u(4,:));
    
    u_stance_ankle_data_timeseries = timeseries;
    u_stance_ankle_data_timeseries.Time = 0:.00001:i*.0083;
    u_stance_ankle_data_timeseries.Data = u_stance_ankle_data(1:(i*830+1));
    u_stance_knee_data_timeseries = timeseries;
    u_stance_knee_data_timeseries.Time = 0:.00001:i*.0083;
    u_stance_knee_data_timeseries.Data = u_stance_knee_data(1:(i*830+1));
    u_swing_ankle_data_timeseries = timeseries;
    u_swing_ankle_data_timeseries.Time = 0:.00001:i*.0083;
    u_swing_ankle_data_timeseries.Data = u_swing_ankle_data(1:(i*830+1));
    u_swing_knee_data_timeseries = timeseries;
    u_swing_knee_data_timeseries.Time = 0:.00001:i*.0083;
    u_swing_knee_data_timeseries.Data = u_swing_knee_data(1:(i*830+1));
        
    clear u(1,:) u(2,:) u(3,:) u(4,:)
    clear u_stance_ankle_data u_stance_knee_data u_stance_hip_data u_swing_ankle_data u_swing_knee_data u_swing_hip_data
    
    sim(Plant);
    
    % Save all the variable after one iteration simulation of plant model into a file
    filename = ['ds_plant_v2_1_iter_' num2str(i+1)];
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
    
    filename = ['ds_nextoptIC_iter_' num2str(i+1)];
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
        
    filename = ['ds_controlinputhistory_iter_' num2str(i+1)];
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    toc

end

%% plot sim results
L = (0.00001*length(stance_ankle_p_out));
t = 0.00001:0.00001:L;

% figure 1 - joint angle profiles 
figure(1)
subplot(231)
plot(t,stance_ankle_p_out)
title('Stance Ankle Position')
xlabel('time (s)')
ylabel('Ankle Angle (degrees)')
grid on

subplot(232)
plot(t,-stance_knee_p_out)
title('Stance Knee Position')
xlabel('time (s)')
ylabel('Knee Angle (degrees)')
grid on

subplot(233)
plot(t,stance_hip_p_out)
title('Stance Hip Position')
xlabel('time (s)')
ylabel('Hip Angle (degrees)')
grid on

subplot(234)
plot(t,swing_ankle_p_out)
title('Swing Ankle Position')
xlabel('time (s)')
ylabel('Ankle Angle (degrees)')
grid on

subplot(235)
plot(t,-swing_knee_p_out)
title('Swing Knee Position')
xlabel('time (s)')
ylabel('Knee Angle (degrees)')
grid on

subplot(236)
plot(t,swing_hip_p_out)
title('Swing Hip Position')
xlabel('time (s)')
ylabel('Hip Angle (degrees)')
grid on