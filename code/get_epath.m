function epath = get_epath(particles, epath, NPARTICLES)

vth = 1; % focus on the first vehicle

% vehicle state estimation result
xvp = [particles.xv];
xfp = [];                % feature vehicle
for i = 1:NPARTICLES
    xfp = [xfp; particles(i).xf];
end

w = [];
for i=1:NPARTICLES, w = [w particles(i).w(vth)]; end 
ws= sum(w); w= w/ws; % normalize

% weighted mean vehicle pose
xvmean= 0;
for i=1:NPARTICLES
    xvmean = xvmean+w(i)*xvp(:,i);               
end

nf = 3;            % number of feature vehicle
nv = 2;            % number of feature state
xvfmean = zeros(nv,nf);
for i = 1:nf
    xvt = xfp(:,i);
    for j = 1 : NPARTICLES
        xvfmean(:,i) = xvfmean(:,i) + w(j) *xvt(2*j-1:2*j); 
    end
end


% keep the pose for recovering estimation trajectory
epath.v1=[epath.v1 xvmean]; 
epath.v2=[epath.v2 xvfmean(:,1)];
epath.v3=[epath.v3 xvfmean(:,2)];
epath.v4=[epath.v4 xvfmean(:,3)];

