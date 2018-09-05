function data = uslam

clear; close; setconfigfile; h = setup_animations(road,vtx);
particles = initialize_particles(NPARTICLES,road,vtx);
profile off;
profile on -detail builtin

timeframe = (timestep-3)/3;  % 3 timestep in one frame
for d =1:timeframe % for all sampling steps (use faster way)           
    step = [(d*3-2)+1:(d*3)+1];    % start from second time
    for k = 1:3
        t = step(k);
        % Predict vehicle state
        for i=1:NPARTICLES
            particles(i)= predictACFRu(particles(i),V(:,t),Qe,dt,n_r,lambda_r,wg_r,wc_r);
        end
        
        % Get observation
        for i=1:NPARTICLES
            particles(i).zf = dist(:,t).';
        end

        % Known map features
        for i=1:NPARTICLES
            %if ~isempty(particles(i).zf)                      
                % Sample from optimal proposal distribution
                particles(i) = sample_proposaluf(particles(i),particles(i).zf,Re,n_aug,lambda_aug,wg_aug,wc_aug);                                                
                % Map update
                particles(i)= feature_updateu(particles(i),particles(i).zf,Re,n_f_a,lambda_f_a,wg_f_a,wc_f_a);                        
            %end
        end 

        % use the first particle for drawing the laser line   
        plines= make_laser_lines(particles,particles(1).xv);                               

        % Resampling *before* computing proposal permits better particle diversity
        particles= resample_particles(particles, NEFFECTIVE);            

        % When new features are observed, augment it to the map
        %for i=1:NPARTICLES        
        %    if ~isempty(particles(i).zn)
        %        if isempty(particles(i).zf) % sample from proposal distribution (if we have not already done so above)
        %            particles(i).xv= multivariate_gauss(particles(i).xv, particles(i).Pv, 1);
        %            particles(i).Pv= eps*eye(3);
        %        end                        
        %        particles(i)= add_feature(particles(i), particles(i).zn,Re);
        %    end
        %featurecountnum = i_obs;            
        %break;
        %end            


        % Plot
        epath = get_epath(particles, epath, NPARTICLES);
        do_plot(h, particles, plines, epath);        
    end
end
profile report

data= particles;

% 
%

function p = initialize_particles(np,road,xvt)
for i=1:np
    p(i).w= 1/np; % initial particle weight
    p(i).xv= road(:,1); % initial vehicle pose
    p(i).Pv= 1*eye(2); % initial robot covariance that considers a numerical error
    p(i).Kaiy= []; % temporal keeping for a following measurement update
    p(i).xf= xvt; % feature mean states
    for j=1:size(xvt,2), p(i).Pf(:,:,j)= 1*eye(2); end % feature covariances    
    p(i).zf= []; % known feature locations
    p(i).idf= []; % known feature index
    p(i).zn= []; % New feature locations  
    p(i).xvl = []; % last vehicle state
end

function p= make_laser_lines (particle,xv)
if isempty(particle), p=[]; return, end
len = size(particle,2);
xf = [particle.xf];
xf = reshape(xf,[],len);
xf = mean(xf,2);
xf = reshape(xf,2,[]);
lnes(1,:)= zeros(1,size(xf,2))+ xv(1);
lnes(2,:)= zeros(1,size(xf,2))+ xv(2);
lnes(3:4,:)= xf;
p= line_plot_conversion (lnes);

function p= make_covariance_ellipses(particle)
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];
p= make_ellipse(particle.xv(1:2), particle.Pv(1:2,1:2) + eye(2)*eps, circ);
lenf= size(particle.xf,2);
if lenf > 0  
    xf= particle.xf;
    Pf= particle.Pf;
    p= [p zeros(2, lenf*(N+2))];
    ctr= N+3;
    for i=1:lenf
        ii= ctr:(ctr+N+1);
        p(:,ii)= make_ellipse(xf(:,i), Pf(:,:,i), circ);
        ctr= ctr+N+2;
    end
end

function p= make_ellipse(x,P,circ)
% make a single 2-D ellipse 
r= sqrtm_2by2(P);
a= r*circ;
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

function h= setup_animations(road,vtx)
figure;
axis([0 60 0 60]);
xlabel('[m]'); ylabel('[m]');
hold on, % axis equal
h.obs= plot(0,0,'g','erasemode','xor'); % observations
h.xfp= plot(0,0,'r.','erasemode','background'); % estimated features (particle means)
h.xvp= plot(0,0,'r.','erasemode','xor'); % estimated vehicle (particles)
h.cov= plot(0,0,'erasemode','xor'); % covariances of max weight particle
h.epath= plot(0,0,'k','erasemode','xor');
h.xvt = plot(vtx(1,:),vtx(2,:),'+c','erasemode','background');
h.realpath = plot(road(1,1:end-3),road(2,1:end-3),'.c','erasemode','background');

function do_plot(h, particles, plines, epath)
xvp = [particles.xv];
xfp = [particles.xf];
w = [particles.w]; 
ii= find(w== max(w)); 
if ~isempty(xvp), set(h.xvp, 'xdata', xvp(1,:), 'ydata', xvp(2,:)), end
if ~isempty(xfp), set(h.xfp, 'xdata', xfp(1,:), 'ydata', xfp(2,:)), end
if ~isempty(plines), set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:)), end
%pcov= make_covariance_ellipses(particles(ii(1)));
%if ~isempty(pcov), set(h.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:)); end
set(h.epath, 'xdata', epath(1,:), 'ydata', epath(2,:))
drawnow
