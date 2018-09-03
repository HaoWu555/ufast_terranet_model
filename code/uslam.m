function data = uslam

rng default;

clear; close; setconfigfile; h= setup_animations(car_es,car);
particles = initialize_particles(NPARTICLES,car_es);
profile off;
profile on -detail builtin

for t=2:timestep % for all sampling steps (use faster way)   
    
    % Predict vehicle state
    for i=1:NPARTICLES
        particles(i)= predictACFRu(particles(i),V(1:2,t),Qe,vehicle,dt,n_r,lambda_r,wg_r,wc_r);
    end

    % Predict feature vehicle state
    for i=1:NPARTICLES
        particles(i)= predict_feature(particles(i),V(3:end,t),Qe,vehicle,dt,n_r,lambda_r,wg_r,wc_r);
    end
    
    % Measurement update
    % Get observation
    for i= 1:NPARTICLES
        particles(i) = data_assosication(particles(i),D(:,t));
    end
    
    % make the laser line
    plines= make_laser_lines(particles(1).xv,particles(1).xf); % use the first particle for drawing the laser line                               

    % Known map features
    vv = [1 2 3 4 1].';  % the order of vehicle used to calculation in vehicle
    for k = 1:size(vv)   % j=1 always be the host vehicle
        j = vv(k);
        for i=1:NPARTICLES                   
                % Sample from optimal proposal distribution
                particles(i) = sample_proposaluf(particles(i),particles(i).zf,Re,n_aug,lambda_aug,wg_aug,wc_aug,j);                                                
                % Map update
                particles(i)= feature_updateu(particles(i),particles(i).zf,Re,n_f_a,lambda_f_a,wg_f_a,wc_f_a,j);                        
        end 
        
        particles= resample_particles(particles, NEFFECTIVE, j);   
    end
    

    % When new features are observed, augment it to the map
    %for i=1:NPARTICLES        
    %    if ~isempty(particles(i).zn)
    %        if isempty(particles(i).zf) % sample from proposal distribution (if we have not already done so above)
    %            particles(i).xv= multivariate_gauss(particles(i).xv, particles(i).Pv, 1);
    %            particles(i).Pv= eps*eye(3);
    %        end                        
    %        particles(i)= add_feature(particles(i), particles(i).zn,Re);
    %    end
    %end
                  
    % Plot
    epath = get_epath(particles, epath, NPARTICLES);
    do_plot(h, particles, plines, epath, car, NPARTICLES);  % the host vehicle is 1st vehicle
end
profile report

data= particles;

% 
%

function p = initialize_particles(np,car_es)
for i=1:np
    for j=1:size(car_es,1)/2, p(i).w(j)= 1/np; end % initial particle weight
    p(i).xv= [car_es(1,1);car_es(2,1)]; % initial vehicle pose
    p(i).Pv= 2*eye(2); % initial robot covariance that considers a numerical error
    p(i).Kaiy= []; % temporal keeping for a following measurement update
    p(i).xf= [car_es(3:2:7,1).'; car_es(4:2:8,1).']; % feature mean states
    for j= 1: size(car_es)/2-1
        p(i).Pf(:,:,j)= 2*eye(2); % feature covariances
    end
    p(i).zf= []; % known feature locations
    p(i).idf= []; % known feature index
    p(i).zn= []; % New feature locations   
end

function p= make_laser_lines (xv,xf)
if isempty(xv), p=[]; return, end
v1 = xv;  % hose vehicle
v2 = xf(:,1); v3 = xf(:,2); v4 = xf(:,3);  % feature vehicle
lnes(1,:) = zeros(1,3) + v1(1);
lnes(2,:) = zeros(1,3) + v1(2);
lnes(3:4,:) = [v2(1) v3(1) v4(1); 
               v2(2) v3(2) v4(2)]; 
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

function h= setup_animations(car_es,car)
figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
axis([400 650 400 750]);
xlabel('[m]'); ylabel('[m]');
hold on,% axis equal
h.obs= plot(0,0,'g','erasemode','xor'); % observations
h.xfp= plot(0,0,'r*','erasemode','background'); % estimated features (particle means)
h.xvp= plot(car_es(1,1),car_es(2,1),'r*','erasemode','xor'); % estimated vehicle (particles)
h.cov= plot(0,0,'erasemode','xor'); % covariances of max weight particle
h.epath1= plot(0,0,'k','erasemode','xor'); 
h.epath2= plot(0,0,'k','erasemode','xor'); 
h.epath3= plot(0,0,'k','erasemode','xor'); 
h.epath4= plot(0,0,'k','erasemode','xor'); 
h.realpath1 = plot(car(1,1),car(2,1),'b.');
h.realpath2 = plot(car(3,1),car(4,1),'k.');
h.realpath3 = plot(car(5,1),car(6,1),'b.');
h.realpath4 = plot(car(7,1),car(8,1),'k.');


function do_plot(h, particles, plines, epath, car, NPARTICLES)
% vth is the host vehilce
xvp = [particles.xv];
xfp = [particles.xf];
w = [];
for i=1:NPARTICLES, w = [w particles(i).w(1)]; end 
ii= find(w== max(w));  % plot the most value important particles
if ~isempty(xvp), set(h.xvp, 'xdata', xvp(1,:), 'ydata', xvp(2,:)), end
if ~isempty(xfp), set(h.xfp, 'xdata', xfp(1,:), 'ydata', xfp(2,:)), end
if ~isempty(plines), set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:)), end
%pcov= make_covariance_ellipses(particles(ii(1)));
%if ~isempty(pcov), set(h.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:)); end
set(h.epath1, 'xdata', epath.v1(1,:), 'ydata', epath.v1(2,:))
set(h.epath2, 'xdata', epath.v2(1,:), 'ydata', epath.v2(2,:))
set(h.epath3, 'xdata', epath.v3(1,:), 'ydata', epath.v3(2,:))
set(h.epath4, 'xdata', epath.v4(1,:), 'ydata', epath.v4(2,:))
set(h.realpath1, 'xdata', car(1,:),'ydata',car(2,:))
set(h.realpath2, 'xdata', car(3,:),'ydata',car(4,:))
set(h.realpath3, 'xdata', car(5,:),'ydata',car(6,:))
set(h.realpath4, 'xdata', car(7,:),'ydata',car(8,:))
drawnow
