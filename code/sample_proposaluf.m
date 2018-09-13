function particle= sample_proposaluf(particle,zf,R,n,lambda,wg,wc)
% Compute proposal distribution and then sample from it

xv_= particle.xv; % predictive vehicle state
Pv_= particle.Pv; % predictive vehicle state error covariance
xf = particle.xf;
dimz = size(zf,2);
xvl = particle.xvl; 
Pvl = particle.Pvl;


%lenidf= length(idf); % number of currently observed features
dimv = size(xv_,1); % vehicle state dimension
dimf = size(xf,1); % feature vehicle state dimension
dimm = size(zf,1); % measurement state dimension

% for batch update
z_hat= zeros(dimm*dimz,1); % predictive observation
z= zeros(dimm*dimz,1); % sensory observation
A= zeros(dimm*dimz,2*n+1); % stack of innovation covariance for vehicle uncertainty
wc_s= sqrt(wc);

for i=1:dimz
    xfi=particle.xf(:,i);  % get j-th feature mean
    Pfi=particle.Pf(:,:,i); % get j-th feature cov.
    z(i,1) = zf(:,i);        % stack of sensory observations
    
    % state augmentation
    x_aug = [xv_ ; xfi];
    P_aug = [Pv_ zeros(dimv,dimf) ; zeros(dimf,dimv) Pfi]; 
   
    xl_aug = [xvl ; xfi];
    Pl_aug = [Pvl zeros(dimv,dimf) ; zeros(dimf,dimv) Pfi]; 
    
    % set sigma points
    Ps = (n+lambda)*(P_aug) + eps*eye(n);   Psl = (n+lambda)*(Pl_aug) + eps*eye(n); 
    Ss = chol(Ps)';                         Ssl = chol(Psl)';
    Ksi = zeros(n,2*n+1);                   Ksil = zeros(n,2*n+1);
    Ksi(:,1) = x_aug;                       Ksil(:,1) = xl_aug;
    for k=1:n
        Ksi(:,k+1) = x_aug + Ss(:,k);
        Ksi(:,k+1+n) = x_aug - Ss(:,k);    
        
        Ksil(:,k+1) = xl_aug + Ssl(:,k);
        Ksil(:,k+1+n) = xl_aug - Ssl(:,k);    
    end
    % passing through observation model
    Ai = zeros(dimm,2*n+1);
    z_hati = 0; % predicted observation ('dimf' by 1)    
    for k=1:(2*n+1) % pass the sigma pts through the observation model
        d= Ksi(dimv+1:dimv+dimf,k) - Ksi(1:dimv,k);
        r= sqrt(d(1)^2 + d(2)^2); % range
        
        dl= Ksil(dimv+1:dimv+dimf,k) - Ksil(1:dimv,k);
        rl= sqrt(dl(1)^2 + dl(2)^2); % range
        
        Ai(:,k)= [r-rl]; % bearing  **do not use pi_to_pi here** 
        z_hati = z_hati + wg(k)*Ai(:,k);  % predictive observation         
    end    
    z_hati_rep= repmat(z_hati,1,2*n+1);
    A(i,:)= (Ai - z_hati_rep); 
    for k=1:(2*n+1)
        A(i,k) = A(i,k) * wc_s(k);
    end    
    z_hat(i,1)= z_hati;  
end

% augmented noise matrix
R_aug = zeros(dimm*dimz,dimm*dimz);
for i=1:dimz
    R_aug(i,i)= R;
end

% innovation covariance (ISSUE)
S=A*A'; % vehicle uncertainty + map + measurement noise
S=(S+S')*0.5 + R_aug;  % make symmetric for better numerical stability

% cross covariance: considering vehicle uncertainty
X = zeros(dimv,2*n+1); % stack
for k=1:(2*n+1)
    X(:,k)= wc_s(k) * (Ksi(1:2,k) - xv_);     
end

U= X*A'; % cross covariance ('dimv' by 'dimf*lenidf')

% Kalman gain
K = U / S;    

% innovation ('dimf*lenidf' by 1) 
v = z-z_hat;

% standard Kalman update
xv = xv_ + K*v;
Pv = Pv_ - K*U'; 

% compute weight (parallel process) : ERB for SLAM problem
Lt= S; % square matrix of 'dimf*lenidf'
den= sqrt(2*pi*det(Lt));
num= exp(-0.5 * v' / Lt * v);
w = num/den;
particle.w = particle.w * w;

% sample from proposal distribution
%xvs = multivariate_gauss(xv,Pv,1); 
%particle.xv = xvs;
%particle.Pv= eye(2)*eps; % initialize covariance 

particle.xv = xv;
particle.Pv = Pv;

%particle.xvl = xvl; 
%particle.Pvl = eye(2)*eps;

%
%

function x= pi_to_pi(x)
for i=1:size(x,2)
    if x(i) > pi
        while x(i) > pi
            x(i)= x(i) - 2*pi;
        end
    elseif x(i) < -pi
        while x(i) < -pi
            x(i)= x(i) + 2*pi;
        end
    end
end
