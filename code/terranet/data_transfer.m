clear all

% car real position
load car.mat
% load the car velocity
load velocity.mat
V = velocity.';
% load car estimation position
load car_es.mat
% load distance
load distance.mat
d = distance.';

D = [d(1:3,:); d(1,:); d(4:5,:); d(2:2:6,:); d(3,:); d(5:6,:)];

vx1 = V(1,:);
vy1 = V(2,:);
vx2 = V(3,:);
vy2 = V(4,:);
vx3 = V(5,:);
vy3 = V(6,:);
vx4 = V(7,:);
vy4 = V(8,:);

velocity = [vx1;vy1;vx2;vy2;vx3;vy3;vx4;vy4];

G1 = atan2(vy1, vx1);
G2 = atan2(vy2, vx2);
G3 = atan2(vy3, vx3);
G4 = atan2(vy4, vx4);

Geering = [G1; G2; G3; G4];

Geering_rate = diff(Geering,1,2);
Geering_rate = [Geering(:,1) Geering_rate];

[a,b] = size(Geering);
Geering = Geering + randn(a,b)* (0.1/180*pi);


v1 = sqrt(vx1.^2 + vy1.^2);
v2 = sqrt(vx2.^2 + vy2.^2);
v3 = sqrt(vx3.^2 + vy3.^2);
v4 = sqrt(vx4.^2 + vy4.^2);

velocity = [v1; v2; v3; v4];
