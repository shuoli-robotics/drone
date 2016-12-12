function [ d_states ] = drone_dynamics( states,omega )
% 
%   
global m I k_F k_M L g R_d_angle current_angular_velocity_accel flag_drag
global flag_model_uncertainty

%% model uncertainty
if flag_model_uncertainty == 1
    Q_pqr = diag(0.02*[1 1 1]).^2;
    mu_pqr = [0 0 0];
else
    Q_pqr = 0;
    mu_pqr = [0 0 0];
end

x_earth = states(1);
y_earth = states(2);
z_earth = states(3);
v_x_body = states(4);
v_y_body = states(5);
v_z_body = states(6);
phi = states(7);
theta = states(8);
psi= states(9);
p = states(10);
q = states(11);
r = states(12);

F = (omega(1)^2+omega(2)^2+omega(3)^2+omega(4)^2)*(-k_F);
M = [(-omega(2)^2+omega(4)^2)*k_F*L (omega(1)^2-omega(3)^2)*k_F*L ...
    (-k_M*omega(1)^2+k_M*omega(2)^2-k_M*omega(3)^2+k_M*omega(4)^2)]';


R_d_angle = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

R_B_E = [cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) ...
    cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);...
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
      sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);...
       -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

 posistion = [x_earth y_earth z_earth]';
 velocity_body = [v_x_body v_y_body v_z_body]' ;
 angle = [phi theta psi]';
 Omega = [p q r]';
 
 d_position = R_B_E * velocity_body;
 d_velocity_body = (R_E_B*[0 0 g]'*m + [0 0 F]')/m -cross(Omega,velocity_body)+ flag_drag* drag_model(states,F);
 d_angle = R_d_angle*Omega;
 d_Omega = inv(I)*(M-cross(Omega,I*Omega))+[mvnrnd(mu_pqr,Q_pqr,1)]';
 
 current_angular_velocity_accel = d_Omega;
 
 d_states = [d_position;d_velocity_body;d_angle;d_Omega];
end

function [D] = drag_model(states,T)

a_bla = 0.0476;
k_par = 0.0036;

velocity_body = [states(4) states(5) states(6)]';
A_bla = [a_bla 0 0;0 a_bla 0; 0 0 0];

D_bla_body = T * A_bla * velocity_body;
D_par_body = k_par * norm(velocity_body) * velocity_body;

D = D_bla_body + D_par_body;
end
