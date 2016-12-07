function DFx = kf_calcF_x()%t, x, u)
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

syms u_n_s theta_s v_n_s phi_s w_n_s psi_s w_n_s x y z

%x_dot(i) = (u_n_s*cos(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*cos(psi_s)-( v_n_s*cos(phi_s)-w_n_s*sin(phi_s))*sin(psi_s);
%y_dot(i) = (u_n_s*cos(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*sin(psi_s)+( v_n_s*cos(phi_s)-w_n_s*sin(phi_s))*cos(psi_s);
%z_dot(i) = -u_n_s*sin(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*cos(theta_s);

DFx = jacobian([(u_n_s*cos(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*cos(psi_s)-( v_n_s*cos(phi_s)-w_n_s*sin(phi_s))*sin(psi_s),
(u_n_s*cos(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*sin(psi_s)+( v_n_s*cos(phi_s)-w_n_s*sin(phi_s))*cos(psi_s),
 -u_n_s*sin(theta_s)+( v_n_s*sin(phi_s)+w_n_s*cos(phi_s))*cos(theta_s)],[u_n_s, theta_s, v_n_s, phi_s, w_n_s, psi_s, w_n_s]);

%{
result = jacobian([x*y*z,y^2,x + z], [x, y, z])

x = 1;
y = 2;
z = 4;

u_n_s = 100;
theta_s = 0.01;
v_n_s = 10;
phi_s = 0.001;
w_n_s = 1;
psi_s = 0.001;
%}
%eval(result)
%eval(DFx)

end

