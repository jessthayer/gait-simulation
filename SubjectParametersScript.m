%% Patient Data Model Creation
%allows input of subject parameters to generate .mat file

%% model name and subject id
%  ***EDIT THESE PARAMETERS ONLY***
id = 02;
param.name = ['SubjectID_',num2str(id)];
param.note = 'Thayer Subject 2, 2019';

%input anthropometric measurements
param.gender = 'female';
param.age = '38';
param.height = 1.78; % (m)
param.weight = 78.02; % (kg)
param.orthosisWeight = 1; % (kg)
param.legLength = 1.01; %ASIS to lateral malleoli (m)
param.thighLength = 0.60; %greater trochanter to lateral femoral epicondyle (m)
param.kneeDiam = 0.1165; %knee diameter (m)
param.ankleDiam = 0.07; %ankle diameter (m)
param.interASIS = 0.32; %ASIS to ASIS (m)
param.thighProx = 0.66; %thigh proximal circumference (m)
param.thighDist = 0.45; %thigh distal circumference (m)
param.malHeight = 0.07; %height of malleolus\ankle joint from ground (m)
param.footLength = 0.25; %length of foot, measured from AFO (m)
param.rockerLength = 0.185; %length from heel to ball of foot, measured from AFO (m)
param.footWidth = 0.10; %width of ball of foot, measured from AFO (m)

%% create simulation parameters
param.var(1) = "bodymass"; param.val(1) = param.weight;
param.var(2) = "F_C"; param.val(2) = 0.33*param.rockerLength;
param.var(3) = "F_H"; param.val(3) = param.malHeight;
param.var(4) = "F_L"; param.val(4) = param.rockerLength;
param.var(5) = "F_M"; param.val(5) = 0.0145*param.val(1);
param.var(6) = "F_I"; param.val(6) = param.val(5)*(0.475*param.val(3))^2;
param.var(7) = "F_W"; param.val(7) = param.footWidth;
param.var(8) = "S_L"; param.val(8) = param.legLength - param.thighLength;
param.var(9) = "S_R1"; param.val(9) = param.ankleDiam/2;
param.var(10) = "S_R2"; param.val(10) = param.thighDist/(2*pi);
param.var(11) = "S_M"; param.val(11) = 0.0465*param.weight;
param.var(12) = "S_I"; param.val(12) = param.val(11)*(0.302*param.val(8))^2;
param.var(13) = "T_L"; param.val(13) = param.thighLength;
param.var(14) = "T_R1"; param.val(14) = param.thighDist/(2*pi);
param.var(15) = "T_R2"; param.val(15) = param.thighProx/(2*pi);
param.var(16) = "T_M"; param.val(16) = 0.100*param.val(1);
param.var(17) = "T_I"; param.val(17) = param.val(16)*(0.475*param.val(13))^2;
param.var(18) = "HAT_L"; param.val(18) = (0.818-0.530)*param.height;
param.var(19) = "HAT_M"; param.val(19) = 0.678*param.val(1);
param.var(20) = "HAT_I"; param.val(20) = param.val(19)*(0.496*param.val(18))^2;
param.var(21) = "HAT_a"; param.val(21) = 0.2;
param.var(22) = "HAT_b"; param.val(22) = 0.11;


%% save to .mat file to be opened and applied in main script
save(['Param_Subject_',num2str(id)],'param');
