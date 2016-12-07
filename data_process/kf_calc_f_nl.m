function xdot = kf_calc_f_nl(t, x, u)
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

g = 9.19;%9.81;

xdot = zeros(15, 1);

%xdot
xdot(1) = (x(4)*cos(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*sin(x(8)) )*cos(x(9))-( x(5)*cos(x(7))-x(6)*sin(x(7)))*sin(x(9));
%ydot
xdot(2) = (x(4)*cos(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*sin(x(8)) )*sin(x(9))+( x(5)*cos(x(7))-x(6)*sin(x(7)))*cos(x(9));
%zdot
xdot(3) = -x(4)*sin(x(8))+( x(5)*sin(x(7))+x(6)*cos(x(7)))*cos(x(8)); 
%udot
xdot(4) = (u(1)-x(10))-g*sin(x(8))+(u(6)-x(15))*x(5)-(u(5)-x(14))*x(6);
%vdot
xdot(5) = (u(2)-x(11))+g*cos(x(8))*sin(x(7))+(u(4)-x(13))*x(6)-(u(6)-x(15))*x(4);
%wdot
xdot(6) = (u(3)-x(12))+g*cos(x(8))*cos(x(7))+(u(5)-x(14))*x(4)-(u(4)-x(13))*x(5);
%phidot
xdot(7) = (u(4)-x(13))+(u(5)-x(14))*sin(x(7))*tan(x(8))+(u(6)-x(15))*cos(x(7))*tan(x(8));
%thetadot
xdot(8) = (u(5)-x(14))*cos(x(7))-(u(6)-x(15))*sin(x(7));
%psidot
xdot(9) = (u(5)-x(14))*(sin(x(7))/cos(x(8)))+(u(6)-x(15))*(cos(x(7))/cos(x(8)));
%x10 = lambda_x
xdot(10) = 0;%constant
%x11 = lambda_y
xdot(11) = 0;
%x12 = lambda_z
xdot(12) = 0;
%x13 = lambda_p
xdot(13) = 0;
%x14 = lambda_q
xdot(14) = 0;
%x15 = lambda_r
xdot(15) = 0;
%x16 = W_x
%xdot(16) = 0;
%x17 = W_y
%xdot(17) = 0;
%x18 = W_z
%xdot(18) = 0;

end

