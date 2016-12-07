function u_k = get_u_k(k,Ax_m,Ay_m,Az_m,p_m,q_m,r_m)
%Extract current u vector from data
%   Detailed explanation goes here
    u_k = zeros(6, 1);
    u_k(1) = Ax_m(k);
    u_k(2) = Ay_m(k);
    u_k(3) = Az_m(k);
    u_k(4) = p_m(k);
    u_k(5) = q_m(k);
    u_k(6) = r_m(k);

end

