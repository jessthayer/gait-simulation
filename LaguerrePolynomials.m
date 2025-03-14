%% Laguerre Polynomial Development
%find polyfit constants to give best initial guess from Winter2009 norm data

close all
%% Single Support Period
%% generate polynomial fit data for subject 4 from normal ambulator
%add winters data
filename = 'C:\Users\jess-local\OneDrive - Marquette University\Research\Normal Gait Data\Winters Gait Data.xlsx';
trajec.ss(:,1) = xlsread(filename,'M29:M56'); %stance ankle (13 timesteps after RHS - 27 timesteps later)
trajec.ss(:,2) = xlsread(filename,'M2:M29'); %swing ankle (RTO - RHS)
trajec.ss(:,3) = xlsread(filename,'N2:N29'); %swing knee (RTO - RHS)
trajec.ss(:,4) = xlsread(filename,'O2:O29'); %swing hip  (RTO - RHS)
trajec.sstime = xlsread(filename,'C2:C29'); %time  (RTO - RHS)

%add polyfit data
%the polyfits act as a baseline trajectory from which the laguerre function
%formulation deviates, they should be a close approximation of winters data
x = 0:0.01:(trajec.sstime(end)+0.02);
lag.ss.polyfit1 = polyfit(trajec.sstime,trajec.ss(:,1),5);
lag.ss.p1 = polyval(lag.ss.polyfit1,x);
lag.ss.polyfit2 = polyfit(trajec.sstime,trajec.ss(:,2),8);
lag.ss.p2 = polyval(lag.ss.polyfit2,x);
lag.ss.polyfit3 = polyfit(trajec.sstime,trajec.ss(:,3),8);
lag.ss.p3 = polyval(lag.ss.polyfit3,x);
lag.ss.polyfit4 = polyfit(trajec.sstime,trajec.ss(:,4),8);
lag.ss.p4 = polyval(lag.ss.polyfit4,x);

%add mpc torques
inter = 'C:\Users\jess-local\OneDrive - Marquette University\Research\Simulations\singleSupportConstraints-04-20-2020 13-10-objfun_v2-inter_v1-plant_v1\SS1\ss1_inter_iter_1';
load(inter)
controlinput = 'C:\Users\jess-local\OneDrive - Marquette University\Research\Simulations\singleSupportConstraints-04-20-2020 13-10-objfun_v2-inter_v1-plant_v1\SS1\ss1_optimizedcontrolinput_iter_1';
load(controlinput)
sdo.setValueInModel(sys,'lagStanceAnkle',u(1,:));
sdo.setValueInModel(sys,'lagSwingAnkle',u(2,:));
sdo.setValueInModel(sys,'lagSwingKnee',u(3,:));
sdo.setValueInModel(sys,'lagSwingHip',u(4,:));
sim(sys)
tq = (0:1:(length(MPCTorqueStanceAnkle)-1))*timeStep;
lag.ss.mpc1 = MPCTorqueStanceAnkle;
lag.ss.mpc2 = MPCTorqueSwingAnkle;
lag.ss.mpc3 = -MPCTorqueSwingKnee;
lag.ss.mpc4 = MPCTorqueSwingHip;
lag.ss.torque1 = torque1;
lag.ss.torque2 = torque2;
lag.ss.torque3 = torque3;
lag.ss.torque4 = torque4;

%plot data and laguerre polyfit constants curve-fits
for i = 1:4
    subplot(str2num(['41' num2str(i)]))
    hold on
    p = eval(['lag.ss.p' num2str(i)]);
    mpc = eval(['lag.ss.mpc' num2str(i)]);
    torque = eval(['lag.ss.torque' num2str(i)]);
    plot(trajec.sstime,trajec.ss(:,i),'.',x,p,tq,mpc,'--') %,x,sun.ss.fit(:,i)
    plot(torque.time,torque.data(:,3))
end

%% generate polynomial fit data for subject 2 from gait analysis data
%add gait study data
clc, clear all

load('gaitAnalysis_Subject_2.mat')
trajec.ss(:,1) = patient_mean.L_ANK_MX(12:50); %stance ankle (12 timesteps after HS - 39 timesteps later)
trajec.ss(:,2) = patient_mean.L_ANK_MX(62:100); %swing ankle (LTO - LHS)
trajec.ss(:,3) = patient_mean.L_KNEE_MX(62:100); %swing knee (LTO - LHS)
trajec.ss(:,4) = patient_mean.L_HIP_MX(62:100); %swing hip  (LTO - LHS)
trajec.sstime = linspace(0,0.39*1.2,39)'; %time  (LTO - LHS)

%add polyfit data
%the polyfits act as a baseline trajectory from which the laguerre function
%formulation deviates, they should be a close approximation of winters data
x = trajec.sstime;
lag.ss.polyfit1 = polyfit(trajec.sstime,trajec.ss(:,1),3);
lag.ss.p1 = polyval(lag.ss.polyfit1,x);
lag.ss.polyfit2 = polyfit(trajec.sstime,trajec.ss(:,2),7);
lag.ss.p2 = polyval(lag.ss.polyfit2,x);
lag.ss.polyfit3 = polyfit(trajec.sstime,trajec.ss(:,3),8);
lag.ss.p3 = polyval(lag.ss.polyfit3,x);
lag.ss.polyfit4 = polyfit(trajec.sstime,trajec.ss(:,4),6);
lag.ss.p4 = polyval(lag.ss.polyfit4,x);

figure(3)
for i = 1:4
    subplot(str2num(['41' num2str(i)]))
    hold on
    p = eval(['lag.ss.p' num2str(i)]);
    plot(trajec.sstime,trajec.ss(:,i),'.',x,p) %,x,sun.ss.fit(:,i)
end

%% generate discrete laguerre functions
%formulation from Wang2009, utilizing a discrete laguerre network state-space representation
a = 0.8;
beta = 1 - a.^2;

A_l = [a 0 0 0 0 0; beta a 0 0 0 0; -a*beta beta a 0 0 0;...
    a^2*beta -a*beta  beta a 0 0; -a^3*beta a^2*beta -a*beta beta a 0;...
    a^4*beta -a^3*beta a^2*beta -a*beta beta a];

L_0 = sqrt(beta)*[1; -a; a^2; -a^3; a^4; -a^5];

t_end = 0.500;
dt = 0.005;
t = 0:dt:t_end;
k_end = t_end/dt;

L(1,:) = L_0;
for k = 1:k_end
    L(k+1,:) = A_l*L(k,:)';
end

figure(2)
plot(t,L)
legend('l_1','l_2','l_3','l_4','l_5','l_6')
ylabel('Amplitude')
xlabel('Sampling Instant, k')
title('Laguerre Functions, M=6, a=0.8')
grid on

