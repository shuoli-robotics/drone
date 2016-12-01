function [flight_data_body_coor ] = data_transform( input_args )
%DATA_TRANSFORM 此处显示有关此函数的摘要
%   此处显示详细说明

flight_data_raw = csvread('roll_3.csv');
flight_data_body_coor = zeros(size(flight_data_raw,1),20);

for i = 1:size(flight_data_raw,1)
    flight_data_body_coor(i,1) = flight_data_raw(i,2);
    flight_data_body_coor(i,2:4) = flight_data_raw(i,12:14); % phi theta psi
    phi = flight_data_raw(i,12);
    theta = flight_data_raw(i,13);
    psi = flight_data_raw(i,14);
    
    R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
 
    flight_data_body_coor(i,5:7) = (flight_data_raw(i,6:8)*0.0009766 );  % ax ay az
    flight_data_body_coor(i,8:10) = (R_E_B*flight_data_raw(i,18:20)')'; % vx vy vz
    
    flight_data_body_coor(i,14:16) = flight_data_raw(i,3:5);   % p q r
    flight_data_body_coor(i,17) = flight_data_raw(i,21);       % Thrust command
    flight_data_body_coor(i,18:20) =  flight_data_raw(i,22:24);  % setpoint phi theta psi
    flight_data_body_coor(i,21:24) = flight_data_raw(i,31:34)/60; % actual rotor speed (HZ!!!) 
    flight_data_body_coor(i,25) = sum(7.226e-6*( flight_data_body_coor(i,21:24).^2)-0.0002327*( flight_data_body_coor(i,21:24))+0.008156); % total trust
end



end

