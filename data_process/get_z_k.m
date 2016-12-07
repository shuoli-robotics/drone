function zpred = get_z_k(k,x_gps,y_gps,z_gps,u_gps,v_gps,w_gps,phi_gps,theta_gps,psi_gps,v_tas_m,alpha_m,beta_m)
%Extract current u vector from data
%   Detailed explanation goes here
    zpred = zeros(12, 1);

    zpred(1) = x_gps(k);
    zpred(2) = y_gps(k);
    zpred(3) = z_gps(k);
    zpred(4) = u_gps(k);
    zpred(5) = v_gps(k);
    zpred(6) = w_gps(k);
    zpred(7) = phi_gps(k);
    zpred(8) = theta_gps(k);
    zpred(9) = psi_gps(k);
    zpred(10) = v_tas_m(k);
    zpred(11) = alpha_m(k);
    zpred(12) = beta_m(k);

end

