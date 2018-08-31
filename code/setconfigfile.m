%%% Set environment
format compact;
path(path, './terranet')
plines= zeros(2,1); 
featurecountnum= 1; % initialize laser feature count
epath= [];  % estimation path

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Experiment on terranet model

% car real position
load car.mat
% load the car velocity
load velocity.mat
V = V;
% load car estimation position
load car_es.mat
% load distance
load D.mat
D = D;

timestep = size(V,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% H/W parameter(truck). Keep the values.
dt= 0.1 ; % [s], time interval between control signals
vehicle.L= 2.83; % [m]
vehicle.H= 0.76; % [m]
vehicle.b= 0.5; % [m]
vehicle.a= 3.78; % [m]
veh= [0 -vehicle.H -vehicle.H; 0 -1 1];

% control noises. You can change these values for your application.
sigmaV= 0.5; % [m/s]
Qe = [sigmaV^2 0; 0 sigmaV^2];  % velocity v and geer g

% observation noises. 
sigmaS= 1; % [m]              % distance
%sigmaB= (1*pi/180); % [rad]   % geering
%Re = [sigmaS^2 0; 0 sigmaB^2];
Re = [sigmaS^2];  %speed

% resampling criteria
NPARTICLES= 5; % number of particles(samples, hypotheses) 
NEFFECTIVE= 0.5*NPARTICLES; % minimum number of effective particles

% data association - innovation gates (Mahalanobis distance)
%GATE_REJECT= 5.991; % maximum distance for association
%GATE_AUGMENT_NN= 2000; % minimum distance for creation of new feature
%GATE_AUGMENT= 100; % minimum distance for creation of new feature (100)

% parameters related to SUT
dimv= 2; % (x, y)
dimQ= 2; % (control value dimension: velocity vx and vy)
dimf= 2; % feature state (x y)
dimz= 1; % number of measurement sensor state,(distance)

% vehicle update
n_aug=dimv+dimQ; 
alpha_aug=0.9; beta_aug=2; kappa_aug=0;
lambda_aug = alpha_aug^2 * (n_aug + kappa_aug) - n_aug; 
lambda_aug=lambda_aug+dimz;
wg_aug = zeros(1,2*n_aug+1); wc_aug = zeros(1,2*n_aug+1);
wg_aug(1) = lambda_aug/(n_aug+lambda_aug);
wc_aug(1) = lambda_aug/(n_aug+lambda_aug)+(1-alpha_aug^2+beta_aug);
for i=2:(2*n_aug+1)
    wg_aug(i) = 1/(2*(n_aug+lambda_aug));    
    wc_aug(i) = wg_aug(i);    
end

% vehicle prediction 
n_r=dimv+dimQ;
alpha_r=0.9;
beta_r=2;
kappa_r=0;
lambda_r = alpha_r^2 * (n_r + kappa_r) - n_r; 
lambda_r= lambda_r+dimz; % should consider dimension of related terms for obtaining equivalent effect with full augmentation
wg_r = zeros(1,2*n_r+1); wc_r = zeros(1,2*n_r+1);
wg_r(1) = lambda_r/(n_r + lambda_r);
wc_r(1) = lambda_r / (n_r+lambda_r) + (1 - alpha_r^2 + beta_r);
for i=2:(2*n_r+1)
    wg_r(i) = 1/(2*(n_r+lambda_r));    
    wc_r(i) = wg_r(i);    
end

%feature updates (augmented state)
n_f_a= dimf + dimz;  
alpha_f_a=0.9; 
beta_f_a=2; 
kappa_f_a=0;
lambda_f_a = alpha_f_a^2 * (n_f_a + kappa_f_a) - n_f_a;
wg_f_a = zeros(1,2*n_f_a+1); wc_f_a = zeros(1,2*n_f_a+1);
wg_f_a(1) = lambda_f_a / (n_f_a + lambda_f_a);
wc_f_a(1) = lambda_f_a / (n_f_a + lambda_f_a) + (1 - alpha_f_a^2 + beta_f_a);
for i=2:(2*n_f_a+1)
    wg_f_a(i) = 1/(2*(n_f_a + lambda_f_a));    
    wc_f_a(i) = wg_f_a(i);    
end

%feature initialization
n_f= dimf;
alpha_f=0.9; 
beta_f=2; 
kappa_f=0;
lambda_f = alpha_f^2 * (n_f + kappa_f) - n_f;
wg_f = zeros(1,2*n_f+1); wc_f = zeros(1,2*n_f+1);
wg_f(1) = lambda_f / (n_f + lambda_f);
wc_f(1) = lambda_f / (n_f + lambda_f) + (1 - alpha_f^2 + beta_f);
for i=2:(2*n_f+1)
    wg_f(i) = 1/(2*(n_f+lambda_f));    
    wc_f(i) = wg_f(i);    
end

epath.v1 = []; epath.v2 = []; epath.v3 = []; epath.v4 = []; 