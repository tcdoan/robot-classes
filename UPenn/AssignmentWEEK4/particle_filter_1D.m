clc
clear all
close all

x =  0.5;       % Initial true state
sigma_x = 0.5;  % noise of the state update model...
sigma_z = 0.5;  % observation noise
T = 100;        % Number of iterations
N = 20;   % Number of particles
V = 1;    % Initial esimate variance
x_P = []; % Particles estimating x
x_P = sqrt(V) .* randn(1,N) + x; % Generate particles from prior dist.

% Models: x = 0.5*x + 25*x/(1 + x^2) + sqrt(x_N)*randn
%         z = x^2/20 + sqrt(x_R)*randn -> measurement noise;
z_out = [x^2 / 20 + sqrt(sigma_z) * randn]; % Actual output vector for measurement values
x_out = [x];  % Actual output vector for measurement values.
x_est = [x];  % Time by time output of the particle filters estimate
x_est_out = [x_est]; % The vector of particle filter estimates.

for t = 1:T
    x = 0.5*x + 25*x/(1 + x^2) + sqrt(sigma_x)*randn;
    z = x^2/20 + sqrt(sigma_z)*randn;
    for i = 1:N
        x_P_update(i) = 0.5*x_P(i) + 25*x_P(i)/(1 + x_P(i)^2) + sqrt(sigma_x)*randn;
        z_update(i) = x_P_update(i)^2/20;
        P_w(i) = (1/sqrt(2*pi*sigma_z)) * exp(-(z - z_update(i))^2/(2*sigma_z));
    end
    
    % Normalize to form a probability distribution (i.e. sum to 1).
    P_w = P_w./sum(P_w);
    
    %% Resampling: From this new distribution, now we randomly sample from it to generate our new estimate particles   
    for i = 1 : N
        x_P(i) = x_P_update(find(rand <= cumsum(P_w),1));
    end
    
    %The final estimate is some metric of these final resampling, such as
    %the mean value or variance
    x_est = mean(x_P);
    
    % Keep track of the 'true' model values
    x_out = [x_out x];
    z_out = [z_out z];
    x_est_out = [x_est_out x_est];
end

t = 0:T;
figure(1);
clf
plot(t, x_out, '.-b', t, x_est_out, '-.r','linewidth',3);
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('t'); ylabel('y');
legend('True model values', 'Filter estimates');