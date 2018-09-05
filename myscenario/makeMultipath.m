% make multipath
Lmain = [road(1,:)-vtx0(1); road(2,:)-vtx0(2)]; 
dist0 = sqrt(Lmain(1,:).^2 + Lmain(2,:).^2);
multi1 = [road(1,:)-vtx1(1); road(2,:)-vtx1(2)]; 
dist1 = sqrt(multi1(1,:).^2 + multi1(2,:).^2);
multi2 = [road(1,:)-vtx2(1); road(2,:)-vtx2(2)]; 
dist2 = sqrt(multi2(1,:).^2 + multi2(2,:).^2);
multi3 = [road(1,:)-vtx3(1); road(2,:)-vtx3(2)]; 
dist3 = sqrt(multi3(1,:).^2 + multi3(2,:).^2);

dist = [dist0; dist1; dist2; dist3];

figure
plot(dist0,'k','LineWidth',2)
hold on
plot(dist1,'r')
plot(dist2,'g')
plot(dist3,'b')
legend('multi1', 'multi2', 'multi3','multi4')
xlabel('time step')
ylabel('Distance')