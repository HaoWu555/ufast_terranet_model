% set the transmitter position 
%tx = [20; 10];
% set the virtual transmitter positon
vtx0 = [20; 10];
vtx1 = [15; 30];
vtx2 = [30; 40];
vtx3 = [50; 15];

vtx = [vtx0 vtx1 vtx2 vtx3];

% set the receiver trajectory (30,15)--> (30,35) --> (45,35) --> (45, 15)

% (30,15)--> (30,35)
v1 = 4 * [0.1 0.1 0.1 0.2 0.3 0.4 0.4 0.4 0.2 0.1 0.1 0.2 0.3 0.4 0.5 0.4 0.3 0.2 0.2 0.1];
lenv1 = length(v1);
s1 = [30;15];
a1 = 15;
b1 = [a1];
for i = 1: lenv1
    a1 = a1 + v1(i);
    b1 = [b1 a1];
end
road1 = [30*ones(1,lenv1+1);b1];
v1x = zeros(1,size(v1,2));
v1y = v1;

% (30,35) --> (45.2,35)
v2 = 4 * [0.1 0.1 0.1 0.2 0.2 0.2 0.3 0.3 0.2 0.2 0.1 0.1 0.3 0.3 0.3 0.2 0.2 0.2 0.1 0.1];
lenv2 = length(v2);
a2 = 30;
b2 = [a2];
for i= 1:lenv2
    a2 = a2 + v2(i);
    b2 = [b2 a2];
end
road2 = [b2; 35*ones(1,lenv2+1)];
v2x = v2;
v2y = zeros(1,size(v2,2));

% (45.2,35) --> (45.2, 15)
v3 = -4 * [0.1 0.1 0.1 0.2 0.3 0.4 0.4 0.4 0.2 0.1 0.1 0.2 0.3 0.4 0.5 0.4 0.3 0.2 0.2 0.1];
lenv3 = length(v3);
a3 = 35;
b3 = [a3];
for i = 1:lenv3
    a3 = a3 + v3(i);
    b3 = [b3 a3];
end
road3 = [45.2*ones(1,lenv3+1); b3];
v3x = zeros(1,size(v3,2));
v3y = v3;

velocity = [[v1x;v1y] [v2x;v2y] [v3x;v3y]];

road = [road1 road2 road3];

figure
f1=plot(road(1,:),road(2,:),'r.','DisplayName','road');
hold on
f2=plot(vtx(1,1),vtx(2,1),'k+','LineWidth',2);
f3=plot(vtx(1,2),vtx(2,2),'r*');
f4=plot(vtx(1,3),vtx(2,3),'g*');
f5=plot(vtx(1,4),vtx(2,4),'b*');
legend([f1,f2,f3,f4,f5], {'road','virtual tx1','virtual tx2', 'virtual tx3','virtual tx4'})
axis([0 60 0 60])
xlabel('X/meters')
ylabel('Y/meters')
hold off











