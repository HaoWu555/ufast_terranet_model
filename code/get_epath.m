function epath = get_epath(particles, epath, NPARTICLES)

% vehicle state estimation result
xvp = [particles.xv];
w = [particles.w]; 
ws= sum(w); w= w/ws % normalize

if isnan(w)
    w = 1/5*ones(1,5);
end

% weighted mean vehicle pose
xvmean= 0;
for i=1:NPARTICLES,
    xvmean = xvmean+w(i)*xvp(:,i);               
end

% keep the pose for recovering estimation trajectory
epath=[epath xvmean]
