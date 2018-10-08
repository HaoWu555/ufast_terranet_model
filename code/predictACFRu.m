function particle = predictACFRu(particle,V,Q,dt,n,lambda,wg,wc)

% INPUTS:
% xv - vehicle pose sample
% Pv - vehicle pose predict covariance
% Note: Pv must be zeroed (or near zero) after each observation. It accumulates the
% vehicle pose uncertainty between measurements.

xv= particle.xv;
Pv= particle.Pv;
dimv= size(xv,1);
dimQ= size(Q,1);

% save the last vehicle state
particle.xvl = xv;
particle.Pvl = Pv;

% state augmentation: process noise only
x_aug = [xv ; zeros(dimQ,1)]; 
P_aug = [Pv zeros(dimv,dimQ) ; zeros(dimQ,dimv) Q]; 

% set sigma points
Z = (n+lambda)*(P_aug) + eps*eye(n);
S = chol(Z)';

Kaix = zeros(n,2*n+1);  % sigma point
Kaix(:,1) = x_aug;

for k=1:n
    Kaix(:,k+1) = x_aug + S(:,k);
    Kaix(:,k+1+n) = x_aug - S(:,k);    
end

Kaiy = zeros(size(xv,1),2*n+1);
for k=1:(2*n+1) % transform sigma pts through the process model
    
    V1 = V(1) + Kaix(3,k); % add process noise of linear speed if exist in Kaix
    V2 = V(2) + Kaix(4,k); % add process noise of linear speed if exist in Kaix
    
    Kaiy(1,k) = Kaix(1,k) + V1*dt;
    Kaiy(2,k) = Kaix(2,k) + V2*dt;
end
 
xv_p = 0;
for k=1:(2*n+1)
    xv_p = xv_p + wg(k)*Kaiy(:,k);
end

Pv_p = 0;
for k=1:(2*n+1)
    Pv_p = Pv_p + wc(k)*(Kaiy(:,k) - xv_p)*(Kaiy(:,k) - xv_p)';
end

particle.xv = xv_p;
particle.Pv = Pv_p;
particle.Kaiy = Kaiy; % keep this for a following measurement update