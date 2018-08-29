function particle = data_assosication(particle,D,G)

vnum = size(G,1);  % vehicle number    
for i = 1: vnum
        %particle.zf(:,i) = [D((vnum-1)*i-(vnum-2):(vnum-1)*i); G(i)];
        particle.zf(:,i) = [D((vnum-1)*i-(vnum-2):(vnum-1)*i)];
end