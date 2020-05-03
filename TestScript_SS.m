%% Single Support Period Test Script
% Test components of main script
%%
clc, clear all
global icID subjID ss_predictionHorizon controlWindow timeStep...
    directory...
    objfun_vers inter_vers plant_vers... 
    trialName sim_file_dir ss_plant_loc ss_inter_loc

%% MPC and Simulation Parameters
% assuming GC time of 1.2s (long for avg GC), SS should be about 38%, DS about 12%
ss_predictionHorizon = 0.4560; %s, will be set as stop time within models as a character vector
controlWindow = 0.0083; %s, consistent with Mo-Cap sampling rate, 120Hz
timeStep = 0.0001; %s

%% *EDIT THESE PARAMETERS ONLY* Define IC and Subject Parameters and Model Versions
subjID = 'Subject_2'; %subject identifier consistent with parameter .mat file
sim_file_dir = pwd; %location of models and cost functions
icID = [sim_file_dir '\IC_ss']; %initial conditions .mat file identifier

%desired test
test.name = 'subjectParameterTest';
test.code = 'ss1';
test.costfxn = 'v2';
test.inter = 'vtest3';
test.plant = 'vtest3';

%desired model versions
objfun_vers = test.costfxn; %version of cost functions to use
inter_vers = test.inter; %version of internal models to use
plant_vers = test.plant; %version of plant models to use

%plant and Internal Model Locations
ss_plant_loc = [sim_file_dir '\ss_plant_' plant_vers '_i'];
ss_inter_loc = [sim_file_dir '\ss_inter_' inter_vers '_i'];

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
