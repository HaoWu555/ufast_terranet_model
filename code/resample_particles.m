function particles = resample_particles(particles, Nmin, j)
% Resample particles if their weight variance is such that N-effective
% is less than Nmin.
% j is the the j-th vehicles

N= length(particles);
w= zeros(1,N);
for i=1:N, w(i)= particles(i).w(j); end
ws= sum(w); w= w/ws;
for i=1:N, particles(i).w(j)= particles(i).w(j) / ws; end

[keep, Neff] = stratified_resample(w);
if Neff <= Nmin
    particles= particles(keep);
    for i=1:N, particles(i).w(j)= 1/N; end
end
