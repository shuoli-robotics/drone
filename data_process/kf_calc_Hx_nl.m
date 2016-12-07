function DHx = kf_calc_Hx_nl(Jac_mat, x, u)
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

%u1 = A_x
%u2 = A_y
%u3 = A_z
%u4 = p
%u5 = q
%u6 = r

 x_s = x(1);
 y_s = x(2);
 z_s = x(3);
 u_n_s = x(4);
 v_n_s = x(5);
 w_n_s = x(6);
 phi_s = x(7);
 theta_s = x(8);
 psi_s = x(9);
 lambda_x = x(10);
 lambda_y = x(11);
 lambda_z = x(12);
 lambda_p = x(13);
 lambda_q = x(14);
 lambda_r = x(15);
 
 DHx = eval(Jac_mat);

end

