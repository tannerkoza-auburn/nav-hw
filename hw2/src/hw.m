%% Navigation - Homework 1 - Tanner Koza
clear
clc
close all


%% Problem 1
% TODO: Refactor this to be cleaner.

walk = readgeotable('strava_lunch-walk.gpx');

% GPS
geoplot(walk.Shape.Latitude, walk.Shape.Longitude)
hold on

% Dead Reckoned
sl = 0.9;
steps = [661 719 777 807 877 898 1200 1226 1529 1603 1673 1699];
stepsDiff = diff(steps);
heading = [78 147 64 72 170 92 14 263 240 290 262];

truthNED = lla2ned([walk.Shape.Latitude, walk.Shape.Longitude, zeros(970,1)],[walk.Shape.Latitude(1) walk.Shape.Longitude(1) 0],'ellipsoid');

N = zeros(1,length(heading)+1);
E = zeros(1,length(heading)+1);

for i = 2:length(heading)
    N(i) = sl*stepsDiff(i-1)*cosd(heading(i-1));
    E(i) = sl*stepsDiff(i-1)*sind(heading(i-1));
end

upN = interp1(0:11,cumsum(N),0:(11/970):11-(11/970));
upE = interp1(0:11,cumsum(E),0:(11/970):11-(11/970));

error = sqrt((truthNED(:,1)'-upN).^2 + (truthNED(:,2)'-upE).^2);

llaDR = ned2lla([cumsum(N') cumsum(E') zeros(1,length(N))'], [walk.Shape.Latitude(1) walk.Shape.Longitude(1) 0], 'ellipsoid');
geoplot(llaDR(:,1),llaDR(:,2))
geobasemap satellite
legend('GPS','Dead Reckoned')
title('GPS vs. Dead Reckoned Walk')

figure
plot(error)
xlabel('GPS Epochs')
ylabel('Euclidean Error (m)')
title('GPS & Dead Reckoned Error')
%% Problem 4

% Initialization
Cba = [0, 0, 1; ...
    0, 1, 0; ...
    -1, 0, 0];
Ccb = [1, 0, -1; ...
    0, 1, 0; ...
    0, 0, 0];
Cdc = [1, 0, 0; ...
    0, 0.5, 0; ...
    0, 0, 2];
Ccd = [0.4330, -0.7718, 0.4656; ...
    0.75, 0.5950, 0.2888; ...
    -0.5, 0.2241, 0.8365];
Cfe = [0.5, -0.1464, 0.8536; ...
    0.5, -0.8536, -0.1464; ...
    -0.7071, 0.5, 0.5];

% Orthogonal Test
orthogA = Cba * Cba';
orthogB = Ccb * Ccb';
orthogC = Cdc * Cdc';
orthogD = Ccd * Ccd';
orthogE = Cfe * Cfe';

% Normal Test
normA = det(Cba);
normB = det(Ccb);
normC = det(Cdc);
normD = det(Ccd);
normE = det(Cfe);

%% Problem 5

% Initialization
C01 = [0, -sqrt(3) / 2, -0.5; ...
    0, -0.5, sqrt(3) / 2; ...
    -1, 0, 0];
v0 = [1; 1; 1];

% Part A
C10 = C01';

xCF1 = [1; 0; 0];
yCF1 = [0; 1; 0];
zCF1 = [0; 0; 1];

xCF10 = C10 * xCF1;
yCF10 = C10 * yCF1;
zCF10 = C10 * zCF1;

% Part B
v1 = C01 * v0;

%% Problem 6

% Part A
angleZA = 90;
Rza = [cosd(angleZA), -sind(angleZA), 0; ...
    sind(angleZA), cosd(angleZA), 0; ...
    0, 0, 1];
angleXA = -180;
Rxa = [1, 0, 0; ...
    0, cosd(angleXA), -sind(angleXA); ...
    0, sind(angleXA), cosd(angleXA)];
CbaA = Rza * Rxa;

% Part B
angleZB = -45;
Rzb = [cosd(angleZB), -sind(angleZB), 0; ...
    sind(angleZB), cosd(angleZB), 0; ...
    0, 0, 1];
angleXB = -90;
Rxb = [1, 0, 0; ...
    0, cosd(angleXB), -sind(angleXB); ...
    0, sind(angleXB), cosd(angleXB)];
CbaB = Rxb * Rzb

%% Problem 7

% Rotation Angles
zTheta = deg2rad(-90); % first rotation (0 to 1)
yTheta = deg2rad(90); % second rotation (0 to 1)
xTheta = deg2rad(-90); % third rotation (0 to 1)

% Rotation Matrices
Rz = [cos(zTheta) sin(zTheta) 0;
        -sin(zTheta) cos(zTheta) 0;
        0 0 1];
Ry = [cos(yTheta) 0 -sin(yTheta);
        0 1 0;
        sin(yTheta) 0 cos(yTheta)];
Rx = [1 0 0;
        0 cos(xTheta) sin(xTheta);
        0 -sin(xTheta) cos(xTheta)];

% NOTE: Ry is pre-multiplied bc it is intrinsic and
% Rx is post-multiplied bc it is extrinsic
R01 = Ry * Rz * Rx; 
R10 = R01';

figure
dcmPlot2(R01)

%% Problem 8

phi = 120;
theta = 45;
psi = -120;

Cz = [cosd(psi), -sind(psi), 0; ...
    sind(psi), cosd(psi), 0; ...
    0, 0, 1];
Cy = [cosd(theta), 0, sind(theta); ...
    0, 1, 0; ...
    -sind(theta), 0, cosd(theta)];
Cx = [1, 0, 0; ...
    0, cosd(phi), -sind(phi); ...
    0, sind(phi), cosd(phi)];
C = Cx * Cy * Cz;

yaw = rad2deg(atan2(C(1,2),C(1,1)))
pitch = rad2deg(asin(C(1,3)))
roll = rad2deg(atan2(C(2,3),C(3,3)))

%% Problem 9

theta = 10; %pi/2 - atan2(4,3);

Rx = [1 0 0;
    0 cosd(theta) -sind(theta);
    0 sind(theta) cosd(theta)]

Ry =[cosd(20) 0 sind(20);
    0 1 0;
    -sind(20) 0 cosd(20)]

Rz = [cosd(30) -sind(30) 0;
    sind(30) cosd(30) 0; ...
    0 0 1];

CBG = Rx * Ry * Rz

% Rx = [1, 0, 0; ...
%     0, cosd(atan2d(4,3)), -sind(atan2d(4,3)); ...
%     0, sind(atan2d(4,3)), cosd(atan2d(4,3))];
% CBA = Rx 