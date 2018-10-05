function particle= feature_updateu(particle,zf,R,n,lambda,wg,wc)
% Having selected a new pose from the proposal distribution, this pose is assumed
% perfect and each feature update may be computed independently and without pose uncertainty.

xv= particle.xv;        
xf= particle.xf;
Pf= particle.Pf;

xvl = particle.xvl;

dimf = size(xf,1);
dimz = size(zf,2);
dimm = size(zf,1);
dimR = size(R,1);

% each feature
for i=1:dimz
    % augmented feature state
    xf_aug= [xf(:,i) ; zeros(dimR,1)];
    Pf_aug= [Pf(:,:,i) zeros(2,dimR); zeros(dimR,2) R];
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
    Z = zeros(dimm,2*n+1);
    for k=1:(2*n+1)
        d= Kai(1:2,k) - xv(1:2);
        dl = Kai(1:2,k) - xvl(1:2);
        r= sqrt(d(1)^2 + d(2)^2);    % range + noise
        rl= sqrt(dl(1)^2 + dl(2)^2); % range + noise 
        dd = r-rl +Kai(3,k);
        % predictive sigma pints
        Z(:,k)= [ dd ];    % bearing + noise  **do not use pi_to_pi here**     
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
        Sigma = Sigma + wc(k)* (Kai(1:2,k) - xf(:,i))*(Z(:,k) - z_hat)' ;
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
    v=zf(:,i) - z_hat;

    Kt = Sigma / St;    
    xf(:,i) = xf(:,i) + Kt * v;
    Pf(:,:,i) = Pf(:,:,i) - Kt * St * Kt';
end

particle.xf= xf;
particle.Pf= Pf;

