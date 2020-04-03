%% MAIN SCRIPT V4
% Generate full Gait Cycle, Combine Joint Angle Trajectories
%%
clc, clear all
global ss_predictionHorizon ds_predictionHorizon controlWindow timeStep...
    directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc

%% MPC and Simulation Parameters
% assuming GC time of 1.2s (long for avg GC), SS should be about 38%, DS about 12%
ss_predictionHorizon = 0.4560; %s, will be set as stop time within models as a character vector
ds_predictionHorizon = 0.1660; %s
controlWindow = 0.0083; %s, consistent with Mo-Cap sampling rate, 120Hz
timeStep = 0.0001; %s

%% *EDIT THESE PARAMETERS ONLY* Define IC and Subject Parameters and Model Versions
subjID = 'Subj04'; %subject identifier consistent with parameter .mat file
icID = 'IC01'; %initial conditions .mat file identifier

%desired model versions
objfun_vers = 'v1'; %version of cost functions to use
inter_vers = 'v1'; %version of internal models to use
plant_vers = 'v1'; %version of plant models to use
sim_file_dir = 'C:\Users\jess-local\OneDrive - Marquette University\Research\Simulations'; %location of models and cost functions

%% Plant and Internal Model Locations
ds_plant_loc = [sim_file_dir '\ds_plant_' plant_vers '_i'];
ds_inter_loc = [sim_file_dir '\ds_inter_' inter_vers '_i'];
ss_plant_loc = [sim_file_dir '\ss_plant_' plant_vers '_i'];
ss_inter_loc = [sim_file_dir '\ss_inter_' inter_vers '_i'];

%% Create Directory for Gait Cycle and Open 
trialName = ['GC' subjID '_' datestr(now,'mm-dd-yyyy HH-MM') '-' icID '-objfun_' objfun_vers '-inter_' inter_vers '-plant_' plant_vers];

cd([sim_file_dir]) %move to directory location before creating new folder
%create GC directory
mkdir(trialName) %create a folder to work in

directory = cd(trialName); %save return filepath as variable

%% Begin GC Loop
try
    %check for completion of DS period 1 or run DS1
    if exist('complete_ds1','var') == 0
        addpath(sim_file_dir)
        run ds1_v4 % run double support period 1
        complete_ds1 = 1;
        cd(subdirectory)
    end
    
    clearvars -except ic ss_predictionHorizon ds_predictionHorizon controlWindow timeStep...
    directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc

    %check for completion of SS period 1 or run SS1
    cd([sim_file_dir '\' trialName])
    if exist('complete_ss1','var') == 0
        addpath(sim_file_dir)
        run ss1_v3 % run double support period 1
        complete_ss1 = 1;
        cd(subdirectory)
    end   
    
    %check for completion of DS period 2 or run DS2
    cd([sim_file_dir '\' trialName])
    if exist('complete_ds2','var') == 0
        addpath(sim_file_dir)
        run ds2_v3 % run double support period 1
        complete_ds2 = 1;
        cd(subdirectory)
    end
    
    %check for completion of SS period 1 or run SS1
    cd([sim_file_dir '\' trialName])
    if exist('complete_ss2','var') == 0
        addpath(sim_file_dir)
        run ss2_v3 % run double support period 1
        complete_ss2 = 1;
        cd(subdirectory)
    end         
       
    clearvars -except directory subdirectory ...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc ss_plant_loc ss_inter_loc
    cd(directory)

catch ME
    %an error will put you here.
    errorMessage = ['Code error(s): ', ME.message '    >>>']; %begin the error message
    for ii = 1:size(ME.stack,1) %go through every row in the stack and print out the function/line
        errorMessage = strcat(errorMessage, ' '' ', ME.stack(ii).name, ' '' at line:', num2str(ME.stack(ii).line));
        errorMessage = strcat(errorMessage, '>>>');
    end
    %set email preferences and authenticate
    setpref('Internet','E_mail','jess.thayer07@gmail.com');
    setpref('Internet','SMTP_Server','smtp.gmail.com');
    setpref('Internet','SMTP_Username','jess.thayer07@gmail.com');
    setpref('Internet','SMTP_Password','ewb2670MU');
    props = java.lang.System.getProperties;
    props.setProperty('mail.smtp.auth','true');
    props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
    props.setProperty('mail.smtp.socketFactory.port','465');
  
    %send error message email
    sendmail('jessica.thayer@marquette.edu','MATLAB ERROR',errorMessage)

end
