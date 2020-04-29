%% SCRIPT TO IMPORT DATA FROM VISUAL3D FOR PROCESSING
%IMPORTS NORMALIZED KINEMATICS FROM n TRIALS OF A SUBJECT AND STORES IN ARRAY

clear all, close all, clc

npatients = 1;

for x = 1:npatients
    close all
    clear fname fid hA hB hC hD hE patientnum v xyz count dataArray
    
fname = 'C:\Users\jess-local\OneDrive - Marquette University\Research\SubjectTesting\Marquette Custom HH Gait PAF_v1.2\Data\First_Last_Subj02\Norm\subj02_norm_sagittalAngleMoment.txt';

patientnum = x;

fid = fopen(fname,'rt');
hA = strtrim(regexp(fgetl(fid),'\t','split'));
hB = strtrim(regexp(fgetl(fid),'\t','split'));
hC = strtrim(regexp(fgetl(fid),'\t','split'));
hD = strtrim(regexp(fgetl(fid),'\t','split'));
hE = strtrim(regexp(fgetl(fid),'\t','split'));
%mat = cell2mat(textscan(fid,repmat('%f',numel(hdr))));

varlist={'Left Ankle','Left Knee','Left Hip','Right Ankle','Right Knee','Right Hip','Right Ankle Moment','Right Knee Moment','Right Hip Moment','Left Ankle Moment','Left Knee Moment','Left Hip Moment'};
varshort={'L_ANK_','L_KNEE_','L_HIP_','R_ANK_','R_KNEE_','R_HIP_','R_ANK_M','R_KNEE_M','R_HIP_M','L_ANK_M','L_KNEE_M','L_HIP_M'};
xyzlist={'X'};

delimiter = '\t';
formatSpec = '%*s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
dataArray = textscan(fid, formatSpec, 'delimiter', delimiter);
fclose(fid);

patient(patientnum).filename=fname;
patient(patientnum).record=hA{1,2};

for v=1:length(varlist)
    for xyz=1:length(xyzlist)
count=1;
for i=2:length(hA)
    if strcmp(hB{i},varlist{v})==1 && strcmp(hE{i},xyzlist{xyz})==1
        temp=strcat('patient(', string(patientnum), ').', strcat(varshort{v},xyzlist{xyz}), '(:,', string(count), ') = dataArray{', string(i-1), '};');
        try
        eval([temp]);
        end
        count=count+1;
    end
end
clear count
    end
end

end

clear varlist
clear varshort
for i=1:npatients
patient_mean(i).filename=patient(i).filename;
patient_mean(i).record=patient(i).record;
end

for x=1:npatients
    close all
    clear fname fid hA hB hC hD hE patientnum v xyz count dataArray
    
fname = 'C:\Users\jess-local\OneDrive - Marquette University\Research\SubjectTesting\Marquette Custom HH Gait PAF_v1.2\Data\First_Last_Subj02\Norm\subj02_norm_spatialTemporal.txt';

patientnum=x;

fid = fopen(fname,'rt');
hA = strtrim(regexp(fgetl(fid),'\t','split'));
hB = strtrim(regexp(fgetl(fid),'\t','split'));
hC = strtrim(regexp(fgetl(fid),'\t','split'));
hD = strtrim(regexp(fgetl(fid),'\t','split'));
hE = strtrim(regexp(fgetl(fid),'\t','split'));

varlist={'Speed','Statures_Per_Second','Stride_Width_Mean','Stride_Length_Mean','Cycle_Time_Mean','Double_Limb_Support_Time_Ave','Left_Step_Length_Mean','Right_Step_Length_Mean','Left_Steps_Per_Minute_Mean','Right_Steps_Per_Minute_Mean','Left_Strides_Per_Minute_Mean','Right_Strides_Per_Minute_Mean','Double_Limb_Support_Time_Ave','Right_Initial_Double_Limb_Support_Time_Mean'};
varshort={'Speed','StaturesPerSec','StrideWidth','StrideLength','CycleTime','DblLimbSupportTime','L_StepLength','R_StepLength','L_StepsPerMin','R_StepsPerMin','L_StridesPerMin','R_StridesPerMin','L_InitDblLimbSupport','R_Ini_DblLimbSupport'};

delimiter = '\t';
formatSpec = '%*s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
dataArray = textscan(fid, formatSpec, 'delimiter', delimiter);
fclose(fid);

patient(patientnum).filename=fname;
patient(patientnum).record=hA{1,2};

for v=1:length(varlist)
count=1;
for i=2:length(hA)
    if strcmp(hB{i},varlist{v})==1
        temp=strcat('patient(', string(patientnum), ').', string(varshort{v}), '(' , string(count), ') = dataArray{', string(i-1), '};');
        try
        eval([temp]);
        end
        count=count+1;
    end
end
clear count
    
end

end




%% calculate means

for x=1:npatients
    close all
    clear fname fid hA hB hC hD hE patientnum v xyz count dataArray
    
    vars = {'L_ANK_X','L_KNEE_X','L_HIP_X','R_ANK_X','R_KNEE_X','R_HIP_X','R_ANK_MX','R_KNEE_MX','R_HIP_MX','L_ANK_MX','L_KNEE_MX','L_HIP_MX','Speed','StaturesPerSec','StrideWidth','StrideLength','CycleTime','DblLimbSupportTime','L_StepLength','R_StepLength','L_StepsPerMin','R_StepsPerMin','L_StridesPerMin','R_StridesPerMin','L_InitDblLimbSupport','R_Ini_DblLimbSupport'};

    for v = 1:length(vars)
        eval(['tlen = size(patient(x).' vars{v} ');'])
        for w = 1:tlen(2)
            for y = 1:tlen(1)
                eval(['tempx(y,w)=str2double(patient(x).' vars{v} '{y,w});'])
            end
        end

        for y = 1:tlen(1)
            eval(['patient_mean(x).', vars{v}, '(y,1) = nanmean(tempx(y,:));']);
        end
            %temp=strcat('patient_mean(', string(patientnum), ').', strcat(varshort{v},xyzlist{xyz}), '(z,', string(1), ') = patient{', string(i-1), '};');
            %try
            %eval([temp]);
            %end
    end
end


%% compute gait parameters    
vars = {'L_ANK_X','L_KNEE_X','L_HIP_X','R_ANK_X','R_KNEE_X','R_HIP_X'};
types={'_max_stance','_max_swing','_min_stance','_min_swing'};

for x=1:npatients
    for v=1:length(vars)
        
    eval(['patient_mean(x).' vars{v} '_max_stance=max(patient_mean(x).' vars{v} '(1:62,1));']);
    eval(['patient_mean(x).' vars{v} '_min_stance=min(patient_mean(x).' vars{v} '(1:62,1));']);
    eval(['patient_mean(x).' vars{v} '_max_swing=max(patient_mean(x).' vars{v} '(63:100,1));']);
    eval(['patient_mean(x).' vars{v} '_min_swing=min(patient_mean(x).' vars{v} '(63:100,1));']);
    
    
    end
end


%% test figure
figure(1)
subplot(321)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_ANK_X(:,1));
            hold on
end
grid on
title('L ANKLE FLEXION')

subplot(323)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_KNEE_X(:,1));
            hold on
end
grid on
title('L KNEE FLEXION')

subplot(325)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_HIP_X(:,1));
            hold on
end
grid on
title('L HIP FLEXION')

subplot(322)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_ANK_X(:,1));
            hold on
end
grid on
title('R ANKLE FLEXION')

subplot(324)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_KNEE_X(:,1));
            hold on
end
grid on
title('R KNEE FLEXION')

subplot(326)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_HIP_X(:,1));
            hold on
end
grid on
title('R HIP FLEXION')

figure(2)
subplot(321)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_ANK_MX(:,1));
            hold on
end
grid on
title('L ANKLE MOMENT')

subplot(323)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_KNEE_MX(:,1));
            hold on
end
grid on
title('L KNEE MOMENT')

subplot(325)
for i=1:length(patient_mean)
    plot(patient_mean(i).L_HIP_MX(:,1));
            hold on
end
grid on
title('L HIP MOMENT')

subplot(322)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_ANK_MX(:,1));
            hold on
end
grid on
title('R ANKLE MOMENT')

subplot(324)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_KNEE_MX(:,1));
            hold on
end
grid on
title('R KNEE MOMENT')

subplot(326)
for i=1:length(patient_mean)
    plot(patient_mean(i).R_HIP_MX(:,1));
            hold on
end
grid on
title('R HIP MOMENT')

%% Save subject data as .mat file
subjID = 'Subject_2';
save(['gaitAnalysis_' subjID],'patient','patient_mean')
