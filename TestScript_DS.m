%% Double Support Period Test Script
% Test components of main script
%%
clc, clear all
global icID ds_predictionHorizon controlWindow timeStep...
    directory...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ds_plant_loc ds_inter_loc

%% MPC and Simulation Parameters
% assuming GC time of 1.2s (long for avg GC), SS should be about 38%, DS about 12%
ds_predictionHorizon = 0.1660; %s
controlWindow = 0.0083; %s, consistent with Mo-Cap sampling rate, 120Hz
timeStep = 0.0001; %s

%% *EDIT THESE PARAMETERS ONLY* Define IC and Subject Parameters and Model Versions
subjID = 'Subject_2'; %subject identifier consistent with parameter .mat file
sim_file_dir = pwd; %location of models and cost functions
icID = [sim_file_dir '\IC_ds_' subjID]; %initial conditions .mat file identifier

%desired test
test.name = 'doubleSupportOptTargets';
test.code = 'ds1';
test.costfxn = 'v1';
test.inter = 'v1';
test.plant = 'v2';

%desired model versions
objfun_vers = test.costfxn; %version of cost functions to use
inter_vers = test.inter; %version of internal models to use
plant_vers = test.plant; %version of plant models to use

%plant and Internal Model Locations
ds_plant_loc = [sim_file_dir '\ds_plant_' plant_vers '_i'];
ds_inter_loc = [sim_file_dir '\ds_inter_' inter_vers '_i'];

%% Create Directory for Gait Cycle and Open 
trialName = [test.name '-' datestr(now,'mm-dd-yyyy HH-MM') '-objfun_' objfun_vers '-inter_' inter_vers '-plant_' plant_vers];

cd([sim_file_dir]) %move to directory location before creating new folder
mkdir(trialName) %create a folder to work in
directory = cd(trialName); %save return filepath as variable

try 
    addpath(directory)
    run(test.code)  % run test
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
