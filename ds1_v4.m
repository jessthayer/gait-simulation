%% DS 1
global ss_predictionHorizon ds_predictionHorizon controlWindow timeStep...
    directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc

cd([sim_file_dir '\' trialName]) %check directory location before creating new folder

mkdir('DS1') %create a folder for gait period
subdirectory = cd('DS1'); %open new folder

%% begin double support 1
delete(gcp('nocreate')) %close any current par pool

diary on %record the history of the command window
diary DSCommandWindowHistory.txt

tic %start timer

parpool %start the parallel computing

%open internal and plant models, set prediction horizon, save to subdirectory
open_system(ds_inter_loc);
save_system(ds_inter_loc, 'ds1_inter');
set_param('ds1_inter','StopTime',num2str(ds_predictionHorizon));
save_system('ds1_inter');

open_system(ds_plant_loc);
save_system(ds_plant_loc, 'ds1_plant');
set_param('ds1_plant','StopTime',num2str(ds_predictionHorizon));
save_system('ds1_plant');

tnow = 0; %set current time to 0

%open internal model, initial conditions inherited
sys = 'ds1_inter';
open_system(sys);

% %set internal model initial conditions
% load([sim_file_dir '\' trialName '\SS1\ss1_plant_end'])
%     sdo.setValueInModel(sys,'p_stance_ankle_i',ic.p(1));
%     sdo.setValueInModel(sys,'w_stance_ankle_i',ic.w(1));
%     sdo.setValueInModel(sys,'p_stance_knee_i',ic.p(2));
%     sdo.setValueInModel(sys,'w_stance_knee_i',ic.w(2));
%     sdo.setValueInModel(sys,'p_stance_hip_i',ic.p(3));
%     sdo.setValueInModel(sys,'w_stance_hip_i',ic.w(3));
%     sdo.setValueInModel(sys,'p_swing_ankle_i',ic.p(4));
%     sdo.setValueInModel(sys,'w_swing_ankle_i',ic.w(4));
%     sdo.setValueInModel(sys,'p_swing_knee_i',ic.p(5));
%     sdo.setValueInModel(sys,'w_swing_knee_i',ic.w(5));
%     sdo.setValueInModel(sys,'p_swing_hip_i',ic.p(6));
%     sdo.setValueInModel(sys,'w_swing_hip_i',ic.w(6));
%     sdo.setValueInModel(sys,'Planar_joint_x_p_i',ic.planar.p(1));
%     sdo.setValueInModel(sys,'Planar_joint_x_v_i',ic.planar.w(1));
%     sdo.setValueInModel(sys,'Planar_joint_y_p_i',ic.planar.p(2));
%     sdo.setValueInModel(sys,'Planar_joint_y_v_i',ic.planar.w(2));
%     sdo.setValueInModel(sys,'Planar_joint_z_p_i',ic.planar.p(3));
%     sdo.setValueInModel(sys,'Planar_joint_z_w_i',ic.planar.w(3));
%     save_system('ds2_inter');

%get design vars for the MPC optimization
p = sdo.getParameterFromModel(sys,{'lagStanceAnkle','lagStanceKnee','lagSwingAnkle','lagSwingKnee',...
    'w_stance_ankle_i', 'w_stance_knee_i','w_stance_hip_i','w_swing_knee_i','w_swing_hip_i'});

%set design vars min and max
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

%set design vars scale
p(1).Scale = 150*ones(1,6);
p(2).Scale = 30*ones(1,6);
p(3).Scale = 8*ones(1,6);
p(4).Scale = 15*ones(1,6);
p(5).Scale = abs(p(5).Value);
p(6).Scale = abs(p(6).Value);
p(7).Scale = abs(p(7).Value);
p(8).Scale = abs(p(8).Value);
p(9).Scale = abs(p(9).Value);

simulator = sdo.SimulationTest(sys); %create simulation tester

%define the optimization algorithm
addpath(sim_file_dir);
costfxn = str2func(['objfunDS_' objfun_vers]);
evalDesign = @(p) costfxn(p,simulator);

%select options for the optimizer
opt = sdo.OptimizeOptions;
opt.MethodOptions.Algorithm = 'interior-point';
opt.MethodOptions.UseParallel = 'always';
opt.MethodOptions.FinDiffType = 'central';
opt.MethodOptions.Hessian = 'bfgs';
opt.MethodOptions.TolFun = 0.1;
opt.MethodOptions.ObjectiveLimit = 5;
opt.MethodOptions.MaxIter = 20;
opt.MethodOptions.TolX = 0.02;
opt.UseParallel = 'always';
opt.OptimizedModel = sys;

[pOpt,optInfo] = sdo.optimize(evalDesign,p,opt); %optimize p, control inputs

save(['ds1_inter_iter_1']); %save internal model iteration

for ii = 1:9
    u(ii,:) = pOpt(ii).Value;
end

save('ds1_optimizedcontrolinput_iter_1','u'); %save optimized control inputs

% %set plant initial conditions
% load([sim_file_dir '\' trialName '\SS1\ss1_plant_end'])
plant = 'ds1_plant';
% sdo.setValueInModel(plant,'p_stance_ankle',ic.p(1));
% sdo.setValueInModel(plant,'p_stance_knee',ic.p(2));
% sdo.setValueInModel(plant,'p_stance_hip',ic.p(3));
% sdo.setValueInModel(plant,'p_swing_ankle',ic.p(4));
% sdo.setValueInModel(plant,'p_swing_knee',ic.p(5));
% sdo.setValueInModel(plant,'p_swing_hip',ic.p(6));
% sdo.setValueInModel(plant,'w_stance_ankle',ic.w(1));
% sdo.setValueInModel(plant,'w_stance_knee',ic.w(2));
% sdo.setValueInModel(plant,'w_stance_hip',ic.w(3));
% sdo.setValueInModel(plant,'w_swing_ankle',ic.w(4));
% sdo.setValueInModel(plant,'w_swing_knee',ic.w(5));
% sdo.setValueInModel(plant,'w_swing_hip',ic.w(6));
% sdo.setValueInModel(plant,'x_planar_p',ic.planar.p(1));
% sdo.setValueInModel(plant,'x_planar_v',ic.planar.w(1));
% sdo.setValueInModel(plant,'y_planar_p',ic.planar.p(2));
% sdo.setValueInModel(plant,'y_planar_v',ic.planar.w(2));
% sdo.setValueInModel(plant,'z_planar_p',ic.planar.p(3));
% sdo.setValueInModel(plant,'z_planar_w',ic.planar.w(3));

%send optimized control input to plant
sdo.setValueInModel(plant,'u_stance_ankle',u(1,:));
sdo.setValueInModel(plant,'u_stance_knee',u(2,:));
sdo.setValueInModel(plant,'u_swing_ankle',u(3,:));
sdo.setValueInModel(plant,'u_swing_knee',u(4,:));
sdo.setValueInModel(plant,'w_stance_ankle',u(5));
sdo.setValueInModel(plant,'w_stance_knee',u(6));
sdo.setValueInModel(plant,'w_stance_hip',u(7));
sdo.setValueInModel(plant,'w_swing_knee',u(8));
sdo.setValueInModel(plant,'w_swing_hip',u(9));

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

sim(plant); %simulate plant model using the opitmized control inputs
save('ds1_plant_iter_1') %save plant iteration

%save video of initial simulation
video = [sim_file_dir '\' trialName '\ds1_sim_initial'];
smwritevideo('ds1_plant',video,'PlaybackSpeedRatio',0.5,'FrameRate',60,'FrameSize',[1280 720])

%save the states of the plant model at the end of the first control window
stance_ankle_p_out1 = stance_ankle_p_out(controlWindow/timeStep + 1);
stance_ankle_w_out1 = stance_ankle_w_out(controlWindow/timeStep + 1);
stance_hip_p_out1 = stance_hip_p_out(controlWindow/timeStep + 1);
stance_hip_w_out1 = stance_hip_w_out(controlWindow/timeStep + 1);
stance_knee_p_out1 = stance_knee_p_out(controlWindow/timeStep + 1);
stance_knee_w_out1 = stance_knee_w_out(controlWindow/timeStep + 1);
swing_ankle_p_out1 = swing_ankle_p_out(controlWindow/timeStep + 1);
swing_ankle_w_out1 = swing_ankle_w_out(controlWindow/timeStep + 1);
swing_knee_p_out1 = swing_knee_p_out(controlWindow/timeStep + 1);
swing_knee_w_out1 = swing_knee_w_out(controlWindow/timeStep + 1);
swing_hip_p_out1 = swing_hip_p_out(controlWindow/timeStep + 1);
swing_hip_w_out1 = swing_hip_w_out(controlWindow/timeStep + 1);
Planar_joint_x_p_out1 = Planar_joint_x_p_out(controlWindow/timeStep + 1);
Planar_joint_x_v_out1 = Planar_joint_x_v_out(controlWindow/timeStep + 1);
Planar_joint_y_p_out1 = Planar_joint_y_p_out(controlWindow/timeStep + 1);
Planar_joint_y_v_out1 = Planar_joint_y_v_out(controlWindow/timeStep + 1);
Planar_joint_z_p_out1 = Planar_joint_z_p_out(controlWindow/timeStep + 1);
Planar_joint_z_w_out1 = Planar_joint_z_w_out(controlWindow/timeStep + 1);

save('ds1_nextoptIC_iter_1','Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
save('ds1_controlinputhistory_iter_1','u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

toc

for i = 1:(floor(ds_predictionHorizon/controlWindow) - 1)
    tic 
    
    %clear all variables except i and global vars    
    clearvars -except i ss_predictionHorizon ds_predictionHorizon controlWindow timeStep...
    directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc
    
    tnow = controlWindow*i; %reset the current simulation time
    
    %open internal MPC model
    sys = 'ds1_inter';
    open_system(sys);
    
    load(['ds1_optimizedcontrolinput_iter_' num2str(i)]); %set MPC design vars
    
    %set design vars max, min, and scale
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
        
    load(['ds1_nextoptIC_iter_' num2str(i)]); %load saved states of the plant model at the end of the previous sample time

    %define internal model IC for next time step
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
    sdo.setValueInModel(sys,'Planar_joint_x_p_i',Planar_joint_x_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_x_v_i',Planar_joint_x_v_out1);
    sdo.setValueInModel(sys,'Planar_joint_y_p_i',Planar_joint_y_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_y_v_i',Planar_joint_y_v_out1);
    sdo.setValueInModel(sys,'Planar_joint_z_p_i',Planar_joint_z_p_out1);
    sdo.setValueInModel(sys,'Planar_joint_z_w_i',Planar_joint_z_w_out1);
    save_system(sys);
    
    simulator = sdo.SimulationTest(sys); %create a simulation tester

    %define the optimization algorithm
    addpath(sim_file_dir);
    costfxn = str2func(['objfunDS_' objfun_vers]);
    evalDesign = @(p) costfxn(p,simulator);
    
    %select options for the optimizer
    opt = sdo.OptimizeOptions;
        opt.MethodOptions.Algorithm = 'interior-point';
        opt.MethodOptions.UseParallel = 'always';
        opt.MethodOptions.FinDiffType = 'central';
        opt.MethodOptions.ObjectiveLimit = 5;
        opt.UseParallel = 'always';
        opt.OptimizedModel = sys;
    
    clear u j
    clear stance_ankle_p_out1 stance_ankle_w_out1 stance_hip_p_out1 stance_hip_w_out1 stance_knee_p_out1 stance_knee_w_out1 swing_ankle_p_out1 swing_ankle_w_out1 swing_hip_p_out1 swing_hip_w_out1 swing_knee_p_out1 swing_knee_w_out1 Planar_joint_x_p_out1 Planar_joint_x_v_out1
    clear Planar_joint_y_p_out1 Planar_joint_y_v_out1 Planar_joint_z_p_out1 Planar_joint_z_w_out1
    clear u_stance_ankle_data_end u_stance_hip_data_end u_stance_knee_data_end u_swing_ankle_data_end u_swing_hip_data_end u_swing_knee_data_end
    clear kk kkk laguerre_coefficient_no_count p1_Max p1_Min p_Max p_Min

    [pOpt,optInfo] = sdo.optimize(evalDesign,p,opt); %optimize

    save(['ds1_inter_iter_' num2str(i+1)]) %save internal model iteration
    
    for ii = 1:4
        u(ii,:) = pOpt(ii).Value;
    end
    
    filename = ['ds1_optimizedcontrolinput_iter_' num2str(i+1)];
    save(filename,'u'); %save optimized control inputs to be applied to plant
    
    %clear required parameters to be able to run the plant model simulation
    clearvars -except i ss_predictionHorizon ds_predictionHorizon controlWindow timeStep...
    directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc
   
    plant = 'ds1_plant'; %load plant
    
    %load initial conditions for next optimization
    load(['ds1_optimizedcontrolinput_iter_' num2str(i+1)]);
    load(['ds1_controlinputhistory_iter_' num2str(i)]);
    
    %simulate model and save the end states to be used as the initial conditions for the next time step
    sdo.setValueInModel(plant,'BeginTimePlantModel',i*controlWindow);
    sdo.setValueInModel(plant,'u_stance_ankle',u(1,:));
    sdo.setValueInModel(plant,'u_stance_knee',u(2,:));
    sdo.setValueInModel(plant,'u_swing_ankle',u(3,:));
    sdo.setValueInModel(plant,'u_swing_knee',u(4,:));
    
    u_stance_ankle_data_timeseries = timeseries;
    u_stance_ankle_data_timeseries.Time = 0:timeStep:i*controlWindow;
    u_stance_ankle_data_timeseries.Data = u_stance_ankle_data(1:(i*(controlWindow/timeStep)+1));
    u_stance_knee_data_timeseries = timeseries;
    u_stance_knee_data_timeseries.Time = 0:timeStep:i*controlWindow;
    u_stance_knee_data_timeseries.Data = u_stance_knee_data(1:(i*(controlWindow/timeStep)+1));
    u_swing_ankle_data_timeseries = timeseries;
    u_swing_ankle_data_timeseries.Time = 0:timeStep:i*controlWindow;
    u_swing_ankle_data_timeseries.Data = u_swing_ankle_data(1:(i*(controlWindow/timeStep)+1));
    u_swing_knee_data_timeseries = timeseries;
    u_swing_knee_data_timeseries.Time = 0:timeStep:i*controlWindow;
    u_swing_knee_data_timeseries.Data = u_swing_knee_data(1:(i*(controlWindow/timeStep)+1));
        
    clear u u_stance_ankle_data u_stance_knee_data u_stance_hip_data u_swing_ankle_data u_swing_knee_data u_swing_hip_data
    
    sim(plant); %simulate plant
    
    %check if toe-off happens within iteration, if so, exit loop
    if lag_toe_zgrf(end) <= 0.01 && length(lag_toe_zgrf) <= ((controlWindow/timeStep)*(i+1)+1)
        break
    end
    
    save(['ds1_plant_iter_' num2str(i+1)]); %save plant iteration
    
    %save final states and control input history
    stance_ankle_p_out1 = stance_ankle_p_out((i+1)*(controlWindow/timeStep)+1);
    stance_ankle_w_out1 = stance_ankle_w_out((i+1)*(controlWindow/timeStep)+1);
    stance_hip_p_out1 = stance_hip_p_out((i+1)*(controlWindow/timeStep)+1);
    stance_hip_w_out1 = stance_hip_w_out((i+1)*(controlWindow/timeStep)+1);
    stance_knee_p_out1 = stance_knee_p_out((i+1)*(controlWindow/timeStep)+1);
    stance_knee_w_out1 = stance_knee_w_out((i+1)*(controlWindow/timeStep)+1);
    swing_ankle_p_out1 = swing_ankle_p_out((i+1)*(controlWindow/timeStep)+1);
    swing_ankle_w_out1 = swing_ankle_w_out((i+1)*(controlWindow/timeStep)+1);
    swing_knee_p_out1 = swing_knee_p_out((i+1)*(controlWindow/timeStep)+1);
    swing_knee_w_out1 = swing_knee_w_out((i+1)*(controlWindow/timeStep)+1);
    swing_hip_p_out1 = swing_hip_p_out((i+1)*(controlWindow/timeStep)+1);
    swing_hip_w_out1 = swing_hip_w_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_x_p_out1 = Planar_joint_x_p_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_x_v_out1 = Planar_joint_x_v_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_y_p_out1 = Planar_joint_y_p_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_y_v_out1 = Planar_joint_y_v_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_z_p_out1 = Planar_joint_z_p_out((i+1)*(controlWindow/timeStep)+1);
    Planar_joint_z_w_out1 = Planar_joint_z_w_out((i+1)*(controlWindow/timeStep)+1);
    
    filename = ['ds1_nextoptIC_iter_' num2str(i+1)];
    save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');
        
    filename = ['ds1_controlinputhistory_iter_' num2str(i+1)];
    save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');
    
    toc
end

%save video of final simulation
video = [sim_file_dir '\' trialName '\ds1_sim_final'];
smwritevideo('ds1_plant',video,'PlaybackSpeedRatio',0.5,'FrameRate',60,'FrameSize',[1280 720])

save('ds1_plant_end') %save end plant iteration for plotting

%add correction to stance_ankle_angle
footLength = sdo.getValueFromModel(plant,'F_L');
correctionAngle = atan2d(lead_toe_z.data(end),footLength);

%save final states as initial conditions file for next period
filename = 'ds1_plant_FC';
    ic.p(1) = stance_ankle_p_out(end) + correctionAngle;
    ic.w(1) = stance_ankle_w_out(end);
    ic.p(2) = stance_knee_p_out(end);
    ic.w(2) = stance_knee_w_out(end);
    ic.p(3) = stance_hip_p_out(end);
    ic.w(3) = stance_hip_w_out(end);
    ic.p(4) = swing_ankle_p_out(end);
    ic.w(4) = swing_ankle_w_out(end);
    ic.p(5) = swing_knee_p_out(end);
    ic.w(5) = swing_knee_w_out(end);
    ic.p(6) = swing_hip_p_out(end);
    ic.w(6) = swing_hip_w_out(end);
    ic.planar.p(1) = Planar_joint_x_p_out(end);
    ic.planar.w(1) = Planar_joint_x_v_out(end);
    ic.planar.p(2) = Planar_joint_y_p_out(end);
    ic.planar.w(2) = Planar_joint_y_v_out(end);
    ic.planar.p(3) = Planar_joint_z_p_out(end);
    ic.planar.w(3) = Planar_joint_z_w_out(end);
    save(filename,'ic')

filename = 'ds1_nextoptIC_end';
save(filename,'Planar_joint_x_p_out1','Planar_joint_x_v_out1','Planar_joint_y_p_out1','Planar_joint_y_v_out1','Planar_joint_z_p_out1','Planar_joint_z_w_out1','stance_ankle_p_out1','stance_ankle_w_out1','stance_hip_p_out1','stance_hip_w_out1','stance_knee_p_out1','stance_knee_w_out1','swing_ankle_p_out1','swing_ankle_w_out1','swing_hip_p_out1','swing_hip_w_out1','swing_knee_p_out1','swing_knee_w_out1');

filename = 'ds1_controlinputhistory_end';
save(filename,'u_stance_ankle_data','u_stance_knee_data','u_stance_hip_data','u_swing_ankle_data','u_swing_knee_data','u_swing_hip_data');

cd(subdirectory)