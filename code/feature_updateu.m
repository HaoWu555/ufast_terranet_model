function particle= feature_updateu(particle,zf,R,n,lambda,wg,wc,j)
% Having selected a new pose from the proposal distribution, this pose is assumed
% perfect and each feature update may be computed independently and without pose uncertainty.

if j==1
    xv= particle.xv;        
    xf= particle.xf;
    Pf= particle.Pf;
    Pv = particle.Pv;
else  % consider the j is the host vehicle, others are the feature vehicles
    xv= particle.xf(:,j-1);
    % host vehicle
    x = [particle.xv particle.xf];
    x(:,j) = []; % delete the host vehicle now
    xf= x;
    % feature vehicle 
    p = zeros(2,2,4);
    p(:,:,1) = particle.Pv; p(:,:,2:size(zf,2)) = particle.Pf;
    Pv = p(:,:,j);
    p(:,:,j) = [];
    Pf = p;
end

%Pf = Pf(1:2,1:2,:);   % covariance without geering
dimf = size(xf,1);  
dimc = size(zf,2);     % number of vehicle totally 
dimv = size(xf,2);
ob = 1;  % observation value only distance

% each feature
zf = zf(:,j);
for i=1:dimv
    % augmented feature state
    xf_aug= [xf(:,i) ; zeros(ob,1)];
    %Pf_aug= [Pf(:,:,i) zeros(2); zeros(2) R];
    Pf_aug= [Pf(:,:,i) zeros(dimf,ob); zeros(ob,dimf) R];
    % disassemble the covariance
    P = (n+lambda)*(Pf_aug) + eps*eye(n);
    S = chol(P)';
    % get sigma points
    Kai = zeros(n,2*n+1);
    Kai(:,1) = xf_aug;
    for k=1:n
        Kai(:,k+1) = xf_aug + S(:,k);
        Kai(:,k+1+n) = xf_aug - S(:,k);    
    end
    
    % transform the sigma points
    Z = zeros(ob,2*n+1);
    bs=zeros(1,2*n+1); % bearing sign    
    for k=1:(2*n+1)
        d= Kai(1:2,k) - xv(1:2);
        r= sqrt(d(1)^2 + d(2)^2) + Kai(3,k); % range + noise
        %bearing= atan2(d(2),d(1));
        %bs(k)=sign(bearing);    
        %if k>1 % unify the sign
        %    if bs(k) ~= bs(k-1);            
        %        if bs(k)<0 && -pi < bearing && bearing < -pi/2, 
        %            bearing=bearing+2*pi;
        %            bs(k)=sign(bearing);    
        %        elseif bs(k)>0 && pi/2 < bearing && bearing < pi,
        %                bearing=bearing-2*pi;
        %                bs(k)=sign(bearing);    
        %        end
        %    end
        %end
        % predictive sigma pints
        %Z(:,k)= [r;       bearing - xv(3) + Kai(4,k)]; % bearing + noise  **do not use pi_to_pi here**     
        Z(:,k)= [r];
    end  
    
    z_hat = 0; % predictive observation
    for k=1:(2*n+1)
        z_hat = z_hat + wg(k)*Z(:,k);
    end    
    
    St = 0; % innovation covariance (sigma pts includes noise)
    for k=1:(2*n+1)
        St = St + wc(k)*(Z(:,k) - z_hat)*(Z(:,k) - z_hat)'; 
    end
    St= (St+St')*0.5; % make symmetric
    
    Sigma = 0; % cross covariance (think why Kai(1:2,k) is used rather than Kai(:,k))
    for k=1:(2*n+1)
        Sigma = Sigma + wc(k)*(Kai(1:2,k) - xf(:,i))*(Z(:,k) - z_hat)';
    end

    % Cholesky update 
%     SChol= chol(St);
%     SCholInv= inv(SChol); % triangular matrix
%     W1 = Sigma / SChol;
%     W= W1 * SCholInv';
%     z_hat(2)=pi_to_pi(z_hat(2));
%     v=zf(:,i) - z_hat;
%     v(2) = pi_to_pi(v(2));
%     xf(:,i) = xf(:,i) + W*v;
%     Pf(:,:,i) = Pf(:,:,i) - W1 * W1'; 

    % standard KF (slightly faster than Cholesky update, the same accuracy)
%     z_hat(2)=pi_to_pi(z_hat(2));
    v=zf(i) - z_hat;   
    %v(2) = pi_to_pi(v(2));

    Kt = Sigma / St;    
    xf(:,i) = xf(:,i) + Kt * v;
    Pf(:,:,i) = Pf(:,:,i) - Kt * St * Kt';
end

if j==1
    particle.xf= xf;
    particle.Pf= Pf;
elseif j==2
    XF = zeros(dimf,dimc-1);
    XF(:,1) = xv ; % save the correspondding feature vehicle state
    XF(:,2:3) = xf(:,2:3);
    particle.xf = XF;
    particle.xv = xf(:,1);
    
    PF = zeros(2,2,3);
    PF(:,:,1) = Pv;
    PF(:,:,2:3) = Pf(:,:,2:3);
    partilce.Pf = PF;
    particle.Pv = Pf(:,:,1);
elseif j==3
    XF = zeros(dimf,dimc-1);
    XF(:,2) = xv ; % save the correspondding feature vehicle state
    XF(:,1) = xf(:,2);
    XF(:,3) = xf(:,3);
    particle.xf = XF;
    particle.xv = xf(:,1);
    
    PF = zeros(2,2,3);
    PF(:,:,2) = Pv;
    PF(:,:,1) = Pf(:,:,2);
    PF(:,:,3) = Pf(:,:,3);
    partilce.Pf = PF;
    particle.Pv = Pf(:,:,1);
elseif j==4
    XF = zeros(dimf,dimc-1);
    XF(:,3) = xv; % save the correspondding feature vehicle state
    XF(:,1:2) = xf(:,2:3);
    particle.xf = XF;
    particle.xv = xf(:,1);
    
    PF = zeros(2,2,3);
    PF(:,:,3) = Pv;
    PF(:,:,1) = Pf(:,:,2);
    PF(:,:,2) = Pf(:,:,3);
    partilce.Pf = PF;
    particle.Pv = Pf(:,:,1);
        
end

