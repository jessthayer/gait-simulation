clc, clear all, close all

%%
numd = [1 -0.1];
dend = conv([1 -0.8],[1 -0.9]);
H = impz(numd,dend);

%% approx a
a = 0.6;
N = 3;

%define laguerre functions
beta = 1 - a.^2;
A_l = [a 0 0; beta a 0; -a*beta beta a];
L_0 = sqrt(beta)*[1; -a; a^2];

k_end = 69;
k = 1:k_end+1;

L(1,:) = L_0;
for i = 1:k_end
    L(i+1,:) = A_l*L(i,:)';
end

%find coefficients for approximation
c = zeros(1,N);
for i = 1:N
    for ii = 1:k_end
        c(i) = H(ii)*L(ii,i)+c(i);
    end
end

%approximate solution
H_model_a = zeros(1,N);
for i = 1:N
    H_model_a = c(i)*L(:,i) + H_model_a;
end

%% approx b
clear L
a = 0.8;
N = 3;

%define laguerre functions
beta = 1 - a.^2;
A_l = [a 0 0; beta a 0; -a*beta beta a];
L_0 = sqrt(beta)*[1; -a; a^2];

k_end = 69;
k = 1:k_end+1;

L(1,:) = L_0;
for i = 1:k_end
    L(i+1,:) = A_l*L(i,:)';
end

%find coefficients for approximation
c = zeros(1,N);
for i = 1:N
    for ii = 1:k_end
        c(i) = H(ii)*L(ii,i)+c(i);
    end
end

%approximate solution
H_model_b = zeros(1,N);
for i = 1:N
    H_model_b = c(i)*L(:,i) + H_model_b;
end

%% approx c
clear L
a = 0.8;
N = 4;

%define laguerre functions
beta = 1 - a.^2;
A_l = [a 0 0 0; beta a 0 0; -a*beta beta a 0; a^2*beta -a*beta  beta a];
L_0 = sqrt(beta)*[1; -a; a^2; -a^3];

k_end = 69;
k = 1:k_end+1;

L(1,:) = L_0;
for i = 1:k_end
    L(i+1,:) = A_l*L(i,:)';
end

%find coefficients for approximation
c = zeros(1,N);
for i = 1:N
    for ii = 1:k_end
        c(i) = H(ii)*L(ii,i)+c(i);
    end
end

%approximate solution
H_model_c = zeros(1,N);
for i = 1:N
    H_model_c = c(i)*L(:,i) + H_model_c;
end

%% plot results
figure (1)
subplot(311)
hold on
plot(H,'k-')
plot(k,H_model_a,'k--')
legend('data','model')
xlabel('Sampling Instant')
ylabel('Impulse Response')
title('Approximation with N=3, a=0.6')

subplot(312)
hold on
plot(H,'k-')
plot(k,H_model_b,'k--')
legend('data','model')
xlabel('Sampling Instant')
ylabel('Impulse Response')
title('Approximation with N=3, a=0.8')

subplot(313)
hold on
plot(H,'k-')
plot(k,H_model_c,'k--')
legend('data','model')
xlabel('Sampling Instant')
ylabel('Impulse Response')
title('Approximation with N=4, a=0.8')

% figure(2)
% plot(k,L)
% legend('1','2','3','4')