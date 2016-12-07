function G = kf_calc_G_nl(x)
%UNTITLED Nonlinear system noise input matrix
%   Detailed explanation goes here

%x1 = x
%x2 = y
%x3 = z
%x4 = u
%x5 = v
%x6 = w
%x7 = phi
%x8 = theta
%x9 = psi
%x10 = lambda_x
%x11 = lambda_y
%x12 = lambda_z
%x13 = lambda_p
%x14 = lambda_q
%x15 = lambda_r
%x16 = W_x
%x17 = W_y
%x18 = W_z

%u1 = A_x
%u2 = A_y
%u3 = A_z
%u4 = p
%u5 = q
%u6 = r

G = [0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     1 0 0   0          -x(6)                 x(5);
     0 1 0  x(6)         0                   -x(4);
     0 0 1 -x(5)         x(4)                   0;
     0 0 0  1    sin(x(7))*tan(x(8)) cos(x(7))*tan(x(8));
     0 0 0  0            cos(x(7))          -sin(x(7)) ;
     0 0 0  0    sin(x(7))/cos(x(8)) cos(x(7))/cos(x(8));
     0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     0 0 0   0           0                      0;
     0 0 0   0           0                      0];

end

