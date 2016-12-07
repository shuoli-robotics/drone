function zpred = kf_calc_h_nl(t, x, u)
%UNTITLED Nonlinear state matrix f
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

g = 9.81;

zpred(1) = x(1);%x
zpred(2) = x(2);%y
zpred(3) = x(3);%z

%u gps
zpred(4) = (x(4)*cos(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*sin(x(8)) )*cos(x(9))-( x(5)*cos(x(7))-x(6)*sin(x(7)))*sin(x(9));
%v gps
zpred(5) = (x(4)*cos(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*sin(x(8)) )*sin(x(9))+( x(5)*cos(x(7))-x(6)*sin(x(7)))*cos(x(9));
%w gps
zpred(6) = -x(4)*sin(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*cos(x(8)); 

zpred(7) = x(7);%phi
zpred(8) = x(8);%theta
zpred(9) = x(9);%psi

zpred(10) = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2));%vtas
zpred(11) = atan(x(6)/x(4));%alpha
zpred(12) = atan(x(5)/sqrt((x(4)^2)+(x(6)^2)));%beta



end

