function particle = predict_feature(particle,V,G,Q,vehicle,dt,n,lambda,wg,wc)

% INPUTS:
% xv - vehicle pose sample
% Pv - vehicle pose predict covariance
% Note: Pv must be zeroed (or near zero) after each observation. It accumulates the
% vehicle pose uncertainty between measurements.

xf= particle.xf;
Pf= particle.Pf;
dimv= size(xf,2); % number of feature vehicle
dims= size(xf,1);  % dimension of feature state
dimQ= size(Q,1);  % control noise value

xf_p = [];
Pf_p = zeros(dims,dims,dimv);

for i= 1:dimv
    % the vehilce we make a prediction
    xv = xf(:,i); 
    Pv = Pf(:,:,i);    
    Vn = V(i);
    Gn = G(i);
    % state augmentation: process noise only
    x_aug = [xv ; zeros(dimQ,1)]; 
    P_aug = [Pv zeros(dims,dimQ) ; zeros(dimQ,dims) Q]; 

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

        Vn = Vn + Kaix(3,k); % add process noise of linear speed if exist in Kaix
        Gn = Gn + Kaix(4,k); % add process noise of linear angle if exist in Kaix

%       Gn = G + Kaix(5,k); % add process noise of steering if exist in Kaix
%       Vc = Vn / (1-tan(Gn)*vehicle.H/vehicle.L); % transform    
%       Kaiy(1,k) = Kaix(1,k) + dt*(Vc*cos(Kaix(3,k)) - Vc/vehicle.L*tan(Gn)*(vehicle.a*sin(Kaix(3,k))+vehicle.b*cos(Kaix(3,k)))); 
%       Kaiy(2,k) = Kaix(2,k) + dt*(Vc*sin(Kaix(3,k)) + Vc/vehicle.L*tan(Gn)*(vehicle.a*cos(Kaix(3,k))-vehicle.b*sin(Kaix(3,k))));
%       Kaiy(3,k) = Kaix(3,k) + Vc*dt*tan(Gn)/vehicle.L;

        Kaiy(1,k) = Kaix(1,k) + Vn*cos(Gn/pi*180)*dt;
        Kaiy(2,k) = Kaix(2,k) + Vn*sin(Gn/pi*180)*dt;
    end

    xv_p = 0;
    for k=1:(2*n+1)
        xv_p = xv_p + wg(k)*Kaiy(:,k);
    end

    Pv_p = 0;
    for k=1:(2*n+1)
        Pv_p = Pv_p + wc(k)*(Kaiy(:,k) - xv_p)*(Kaiy(:,k) - xv_p)';
    end
    
    xf_p = [xf_p xv_p];
    Pf_p(:,:,i) = Pv_p;
end

particle.xf = xf_p;
particle.Pf = Pf_p;
%particle.Kaiy = Kaiy; % keep this for a following measurement update