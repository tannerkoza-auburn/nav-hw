%% Navigation - Homework 1 - Tanner Koza
clear
clc
close all

%% Problem 5

% Initialization
dt = 0.1;
numSamps = 1000;
numSims = 100;
time = dt * (1:numSamps);
bias = 3;
noise = randn(numSamps, numSims);
acc = bias + noise;
vel = zeros(numSamps, numSims);
pos = zeros(numSamps, numSims);

% Mechanization
for i = 2:numSamps

    vel(i, :) = vel(i-1, :) + acc(i, :) * dt;
    pos(i, :) = pos(i-1, :) + vel(i, :) * dt;

end

% Stochastic Values
meanVel = mean(vel, 2);
meanPos = mean(pos, 2);

varVel = var(vel, 0, 2);
varPos = var(pos, 0, 2);

sigmaVel = std(vel, 0, 2);
sigmaPos = std(pos, 0, 2);

% Plotting
figure('Name', 'Velocity Monte Carlo Results')

subplot(2, 1, 1)
plot(time, meanVel, '.')
hold on
plot(time, meanVel+3*sigmaVel, '.r')
plot(time, meanVel-3*sigmaVel, '.r')
title('Monte Carlo Mean Velocity')
legend('Mean Velocity', '\pm3-\sigma Bounds', 'Location', 'best')
xlabel('Time (s)')
ylabel('Velocity (dist./s)')

subplot(2, 1, 2)
plot(time, varVel, '.')
title('Monte Carlo Velocity Variance')
xlabel('Time (s)')
ylabel('Velocity Variance (dist./s)^2')

figure('Name', 'Position Monte Carlo Results')

subplot(2, 1, 1)
plot(time, meanPos, '.')
hold on
plot(time, meanPos+3*sigmaPos, '.r')
plot(time, meanPos-3*sigmaPos, '.r')
title('Monte Carlo Mean Position')
legend('Mean Position', '\pm3-\sigma Bounds', 'Location', 'best')
xlabel('Time (s)')
ylabel('Position (dist.)')

subplot(2, 1, 2)
plot(time, varPos, '.')
title('Monte Carlo Position Variance')
xlabel('Time (s)')
ylabel('Position Variance (dist.)^2')

%% Problem 6

% Answer: Given the body is known to be stationary, the bias can be calculated by
% determining the mean of the 1000 samples.

meanAcc = mean(acc); % bias estimates

meanBiasEstimates = mean(meanAcc);
fprintf('Mean of Bias Estimates: %0.3f\n', meanBiasEstimates)

varBiasEstimates = var(meanAcc);
fprintf('\nVariance of Bias Estimates: %0.3f\n', varBiasEstimates)

sigmaBiasEstimates = std(meanAcc);
fprintf('\n1-sigma of Bias Estimates: %0.3f\n', sigmaBiasEstimates)