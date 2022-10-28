%% Navigation - Homework 3 - Tanner Koza
clear
clc
close all

%% Problem 2

% Initialization
syms theta1(t) theta2(t) a1 d3

C10 = [cos(theta1(t)) -sin(theta1(t)) 0; ...
    sin(theta1(t)) cos(theta1(t)) 0; ...
    0 0 1];
C21 = [cos(theta2(t)) 0 -sin(theta2(t)); ...
    sin(theta2(t)) 0 cos(theta2(t)); ...
    0 -1 0];
C32 = [1 0 0; ...
    0 0 -1; ...
    0 1 0];
R10_0 = [a1 * cos(theta1(t)); a1 * sin(theta1(t)); 0];
R32_2 = [0; 0; d3];

% Part A
C30 = C10 * C21 * C32;

% Part B
C20 = C10 * C21;
R30_0 = R10_0 + C20 * R32_2;

% Part C
C10Dot = diff(C10);
Omega10_0 = C10Dot * C10';
omega10_0 = navtools.skewm_inv(Omega10_0);

C21Dot = diff(C21);
Omega21_1 = C21Dot * C21';

C32Dot = diff(C32);
Omega32_2 = C32Dot * C32';

%% Problem 3

% Initialization
dt = 0.01;
t = 0:dt:10;
epochs = length(t);

phi = zeros(epochs, 1);
theta = zeros(epochs, 1);
psi = zeros(epochs, 1);

for i = 1:epochs

    Cbn = [cos(t(i)), sin(t(i)) * sin(t(i)^2), sin(t(i)) * cos(t(i)^2); ...
        0, cos(t(i)^2), -sin(t(i)^2); ...
        -sin(t(i)), cos(t(i)) * sin(t(i)^2), cos(t(i)) * cos(t(i)^2)];

    phi(i) = atan2(-Cbn(2, 3), Cbn(3, 3));
    theta(i) = asin(-Cbn(1, 3));
    psi(i) = atan2(-Cbn(1, 2), Cbn(1, 1));

end

figure('visible', 'off')
plot(t, rad2deg(phi))
hold on
plot(t, rad2deg(theta))
plot(t, rad2deg(psi))
% title('Euler Angles')
legend('Roll', 'Pitch', 'Yaw', 'Location', 'southwest')
xlabel('Time (s)')
ylabel('Attitude (degs)')
axis padded

matlab2tikz('report/figs/euler.tex', 'showInfo', false)

%% Problem 4

% Part A
syms t

Cbn = [cos(t), sin(t) * sin(t^2), sin(t) * cos(t^2); ...
    0, cos(t^2), -sin(t^2); ...
    -sin(t), cos(t) * sin(t^2), cos(t) * cos(t^2)];
CbnDot = diff(Cbn);

% Part B
% The files for these are called timeRotation.m and timeRotationDot.m.

% Parts C, D, & E
clear Cbn CbnDot

t = [0 0.5 1];
numEpochs = length(t);

Cbn = zeros(3, 3, numEpochs);
CbnDot = zeros(3, 3, numEpochs);
skewbn_n = zeros(3, 3, numEpochs);
omegabn_n = zeros(3, numEpochs);
omegaMag = zeros(1, numEpochs);
theta = zeros(1, numEpochs);
kbn_n = zeros(3, numEpochs);

for i = 1:numEpochs

    Cbn(:, :, i) = timeRotation(t(i));
    CbnDot(:, :, i) = timeRotationDot(t(i));

    skewbn_n(:, :, i) = CbnDot(:, :, i) * Cbn(:, :, i)';
    omegabn_n(:, i) = navtools.skewm_inv(skewbn_n(:, :, i));

    omegaMag(i) = vecnorm(omegabn_n(:, i));

    theta(i) = acos((trace(Cbn(:, :, i)) - 1)/2);
    kbn_n(:, i) = (1 / (2 * sin(theta(i)))) * [Cbn(3, 2, i) - Cbn(2, 3, i); Cbn(1, 3, i) - Cbn(3, 1, i); Cbn(2, 1, i) - Cbn(1, 2, i)];

end

% Part F
dt = 0.1;
t = 0:dt:1;
numEpochs = length(t);

Cbn = zeros(3, 3, numEpochs);
CbnDot = zeros(3, 3, numEpochs);
skewbn_n = zeros(3, 3, numEpochs);
omegabn_n = zeros(3, numEpochs);
omegaMagF = zeros(1, numEpochs);
theta = zeros(1, numEpochs);
kbn_n = zeros(3, numEpochs);

for i = 2:numEpochs

    Cbn(:, :, i) = timeRotation(t(i));
    CbnDot(:, :, i) = (Cbn(:, :, i) - Cbn(:, :, i-1)) / dt;

    skewbn_n(:, :, i) = CbnDot(:, :, i) * Cbn(:, :, i)';
    omegabn_n(:, i) = navtools.skewm_inv(skewbn_n(:, :, i));

    omegaMagF(i) = vecnorm(omegabn_n(:, i));

    theta(i) = acos((trace(Cbn(:, :, i)) - 1)/2);
    kbn_n(:, i) = (1 / (2 * sin(theta(i)))) * [Cbn(3, 2, i) - Cbn(2, 3, i); Cbn(1, 3, i) - Cbn(3, 1, i); Cbn(2, 1, i) - Cbn(1, 2, i)];

end

%% Problem 5

latE = deg2rad(27.98777778);
lonE = deg2rad(86.94444444);
hE = 8850;

rE = llh2xyz(latE, lonE, hE);

llh = xyz2llh(rE(1), rE(2), rE(3));

latE2 = rad2deg(llh(1));
lonE2 = rad2deg(llh(2));
hE2 = llh(3);

% Part E
mu = 3.986004418e14;
J2 = 1.082627e-3;
R_0 = 6378137;
rE2 = llh2xyz(latE,lonE,0);

omega_ie = 7.2992115e-5;
centripEllip = omega_ie^2*[1 0 0; 0 1 0; 0 0 0]*rE2'
centripPeak = omega_ie^2*[1 0 0; 0 1 0; 0 0 0]*rE'

rEVec = [rE(1)*(1-5*(rE(3)/vecnorm(rE))^2); rE(2)*(1-5*(rE(3)/vecnorm(rE))^2);rE(3)*(3-5*(rE(3)/vecnorm(rE))^2)]
gammaPeak = -(mu/vecnorm(rE)^3)*(rE'+(1.5*J2*(R_0^2/vecnorm(rE)^2))*rEVec);
magGammaPeak = vecnorm(gammaPeak);

rEVec2 = [rE2(1)*(1-5*(rE2(3)/vecnorm(rE2))^2); rE2(2)*(1-5*(rE2(3)/vecnorm(rE2))^2);rE2(3)*(3-5*(rE2(3)/vecnorm(rE2))^2)]
gammaE = -(mu/vecnorm(rE2)^3)*(rE2'+(1.5*J2*(R_0^2/vecnorm(rE2)^2))*rEVec);
magGammaE = vecnorm(gammaE)

