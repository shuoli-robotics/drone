function [ collision ] = collision_check( p_rand )
%COLLISION_CHECK �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

x_gate = 2.5;
y_gate = 3;
z_gate = -1.5;

l_gate = 1;

A1 = [x_gate-0.5*l_gate y_gate-0.5*l_gate z_gate-0.5*l_gate];
A2 = [x_gate+0.5*l_gate y_gate-0.5*l_gate z_gate-0.5*l_gate];
A3 = [x_gate+0.5*l_gate y_gate+0.5*l_gate z_gate-0.5*l_gate];
A4 = [x_gate-0.5*l_gate y_gate+0.5*l_gate z_gate-0.5*l_gate];
A5 = [x_gate-0.5*l_gate y_gate-0.5*l_gate z_gate+0.5*l_gate];
A6 = [x_gate+0.5*l_gate y_gate-0.5*l_gate z_gate+0.5*l_gate];
A7 = [x_gate+0.5*l_gate y_gate+0.5*l_gate z_gate+0.5*l_gate];
A8 = [x_gate-0.5*l_gate y_gate+0.5*l_gate z_gate+0.5*l_gate];

theta0 = 30/180*pi;
theta = [theta0-pi -theta0 theta0 pi-theta0]; % theta1 and theta2 are nagetive

phi0 = 30/180*pi;
phi = [pi-phi0 phi0 -phi0 -pi+phi0]; % phi3 and phi4 are nagative
%% check left and right
if p_rand(1) < A4(1)
   % left
  p_rand_y1 = tan(theta(1))*p_rand(1)+A1(2)-tan(theta(1))*A1(1);
  p_rand_y4 = tan(theta(4))*p_rand(1)+A4(2)-tan(theta(4))*A4(1);
  
  if p_rand(2) < p_rand_y4 && p_rand(2) > p_rand_y1
     collision = 1;
     return;
  end
      
elseif p_rand(1) >A2(1)
    %right
       p_rand_y2 = tan(theta(2))*p_rand(1)+A2(2)-tan(theta(2))*A2(1);
       p_rand_y3 = tan(theta(3))*p_rand(1)+A3(2)-tan(theta(3))*A3(1);
       if p_rand(2) < p_rand_y3 && p_rand(2) > p_rand_y2
           collision = 1;
           return;
       end
end

%% check up and down
if (p_rand(2) < A2(2))
    p_rand_z5 = tan(phi(1))*p_rand(2)+A6(3)-tan(phi(1))*A6(2);
    p_rand_z8 = tan(phi(4))*p_rand(2)+A2(3)-tan(phi(4))*A2(2);
    if p_rand(3) >  p_rand_z5 || p_rand(3) <  p_rand_z8
      collision = 1;
           return;
    end
    
elseif (p_rand(2) > A3(2))
    p_rand_z6 = tan(phi(2))*p_rand(2)+A7(3)-tan(phi(2))*A7(2);
    p_rand_z7 = tan(phi(3))*p_rand(2)+A3(3)-tan(phi(3))*A3(2);
    if p_rand(3) < p_rand_z7 ||  p_rand(3) > p_rand_z6
        collision = 1;
           return;
    end
else
    if p_rand(3) < A2(3) || p_rand(3) > A6(3)
        collision = 1;
        return 
    end
end
collision = 0;
end
