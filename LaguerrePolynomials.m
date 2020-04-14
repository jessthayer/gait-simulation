%% Laguerre Polynomial Development
%find polyfit constants to give best initial guess from Winter2009 norm data

clc, clear all, close all
%% Single Support Period
%% generate polynomial fit data for normal ambulator
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
lag.ss.polyfit4 = polyfit(trajec.sstime,trajec.ss(:,4),9);
lag.ss.p4 = polyval(lag.ss.polyfit4,x);

%plot data and laguerre polyfit constants curve-fits
for i = 1:4
    subplot(str2num(['41' num2str(i)]))
    hold on
    p = eval(['lag.ss.p' num2str(i)]);
    plot(trajec.sstime,trajec.ss(:,i),'.',x,p) %,x,sun.ss.fit(:,i)
end

%% generate discrete laguerre functions
%formulation from Wang2009, utilizing a discrete laguerre network
%statespace representation
a = 0.5;
beta = 1 - a.^2;

A_l = [a 0 0 0 0 0; beta a 0 0 0 0; -a*beta beta a 0 0 0;...
    a^2*beta -a*beta  beta a 0 0; -a^3*beta a^2*beta -a*beta beta a 0;...
    a^4*beta -a^3*beta a^2*beta -a*beta beta a];

L_0 = [1; -a; a^2; -a^3; a^4; -a^5];

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

