% angle measurement 

Atan1 = [road(1,:)-vtx0(1); road(2,:)-vtx0(2)]; 
A1 = atan2(Atan1(2,:), Atan1(1,:));

Atan2 = [road(1,:)-vtx1(1); road(2,:)-vtx1(2)]; 
A2 = atan2(Atan2(2,:), Atan2(1,:));

Atan3 = [road(1,:)-vtx2(1); road(2,:)-vtx2(2)]; 
A3 = atan2(Atan3(2,:) , Atan3(1,:));

Atan4 = [road(1,:)-vtx3(1); road(2,:)-vtx3(2)]; 
A4 = atan2(Atan4(2,:), Atan4(1,:));

Angle = [A1; A2; A3; A4];

figure
plot(A1)
hold on 
plot(A2)
plot(A3)
plot(A4)
xlabel('time stamp')
ylabel('degree [rad/s]')