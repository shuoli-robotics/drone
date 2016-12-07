function Jac_mat = kf_calc_jacobian_h()%t, x, u)
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

g = 9.19

syms x_s y_s z_s u_n_s v_n_s w_n_s phi_s theta_s psi_s lambda_x lambda_y lambda_z lambda_p lambda_q lambda_r

Jac_mat = jacobian([x_s;
                    y_s;
                    z_s],[x_s y_s z_s u_n_s v_n_s w_n_s phi_s theta_s psi_s lambda_x lambda_y lambda_z lambda_p lambda_q lambda_r]);


end

