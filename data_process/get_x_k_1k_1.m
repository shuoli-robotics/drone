function x_k_1k_1 = get_x_k_1k_1(k,x_gps,y_gps,z_gps,u_gps,v_gps,w_gps,phi_gps,theta_gps,psi_gps)
%Extract current u vector from data
%   Detailed explanation goes here
    x_k_1k_1 = zeros(18, 1);

    x_k_1k_1(1) = x_gps(k);
    x_k_1k_1(2) = y_gps(k);
    x_k_1k_1(3) = z_gps(k);
    x_k_1k_1(4) = u_gps(k);
    x_k_1k_1(5) = v_gps(k);
    x_k_1k_1(6) = w_gps(k);
    x_k_1k_1(7) = phi_gps(k);
    x_k_1k_1(8) = theta_gps(k);
    x_k_1k_1(9) = psi_gps(k);
    x_k_1k_1(10) = 0.001;%zero inital biases and wind
    x_k_1k_1(11) = 0.001;
    x_k_1k_1(12) = 0.001;
    x_k_1k_1(13) = deg2rad(0.001);
    x_k_1k_1(14) = deg2rad(0.001);
    x_k_1k_1(15) = deg2rad(0.001);
    x_k_1k_1(16) = 10;
    x_k_1k_1(17) = 6;
    x_k_1k_1(18) = 1;

end

