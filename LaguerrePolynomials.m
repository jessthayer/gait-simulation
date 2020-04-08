%% Laguerre Polynomial Development
%find polyfit constants to give best initial guess from Winter2009 norm data

clc, clear all, close all
%% Single Support Period

%add winters data
filename = 'C:\Users\jess-local\OneDrive - Marquette University\Research\Normal Gait Data\Winters Gait Data.xlsx';
trajec.ss(:,1) = xlsread(filename,'M29:M56'); %stance ankle (13 timesteps after RHS - 27 timesteps later)
trajec.ss(:,2) = xlsread(filename,'M2:M29'); %swing ankle (RTO - RHS)
trajec.ss(:,3) = xlsread(filename,'N2:N29'); %swing knee (RTO - RHS)
trajec.ss(:,4) = xlsread(filename,'O2:O29'); %swing hip  (RTO - RHS)
trajec.sstime = xlsread(filename,'C2:C29'); %swing hip  (RTO - RHS)


%add laguerre polyfit data
lag.ss.coeff(1,:) = [23676, -31931, 15253, -3621.6, 268.97, -0.0076232];
lag.ss.coeff(2,:) = [4812.5, -4596.4, 1354.1, -110.89, -8.9175, 1.7133];
lag.ss.coeff(3,:) = [-78996, 67427, -19586, 2161, -128.48, 8.2274];
lag.ss.coeff(4,:) = [-2.1647e+05, 2.0073e+05, -67177, 9816.7, -690.73, 25.178];

x = 0:0.01:trajec.sstime(end)
for ii = 1:4
    lag.ss.fit(:,ii) = lag.ss.coeff(ii,1)*x.^5 + lag.ss.coeff(ii,2)*x.^4 + lag.ss.coeff(ii,3)*x.^3 + ...
        + lag.ss.coeff(ii,4)*x.^2 + lag.ss.coeff(ii,5)*x + lag.ss.coeff(ii,6);
end

%%add sun laguerre polyfit data
sun.ss.coeff(1,:) = [14817.01339121796 -12307.46693630947 2660.607945896111 -392.4653080839798 -22.4679780976277];
sun.ss.coeff(2,:) = [1827062.958846088 -2490254.696117284 1362128.662073937 -378716.5451143687 54333.28215118069];
sun.ss.coeff(3,:) = [-833880.5231337042 699128.4069012062 -179568.6080414001 9158.113454766522 2208.697688414741];
sun.ss.coeff(4,:) = [-165694475.6882803 242689298.7655729 -143233708.2466401 44214448.74291535 -7786254.419671707];

x = 0:0.01:trajec.sstime(end)
for ii = 1:4
    sun.ss.fit(:,ii) = sun.ss.coeff(ii,1)*x.^4 + sun.ss.coeff(ii,2)*x.^3 + sun.ss.coeff(ii,3)*x.^2 + ...
        + sun.ss.coeff(ii,4)*x + sun.ss.coeff(ii,5);
end

%%
%plot data and laguerre polyfit constants curve-fits
for i = 1:4
    subplot(str2num(['41' num2str(i)]))
    hold on
    plot(trajec.sstime,trajec.ss(:,i),'.',x,lag.ss.fit(:,i)) %,x,sun.ss.fit(:,i)
end
