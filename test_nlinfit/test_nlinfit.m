clear
clc

beta = [5 6 7 8 9];


modelfun  = @(beta,x)(beta(1)*x(:,2)-x(:,3)/beta(5))./(1+beta(2)*x(:,1)+beta(3)*x(:,2)+beta(4)*x(:,3));

x = 10*rand(100,4);
y = modelfun(beta,x);  


 
beta0 = 15*[5 5 5 5 5];
beta_estimate = nlinfit(x,y,modelfun,beta0);
beta_estimate1 = lsqcurvefit(modelfun,beta0,x,y);

[flight_data_body_coor] = data_transform();
m = 414/1000;
g = 9.8;
% % body x axis
% x = [flight_data_body_coor(:,3) flight_data_body_coor(:,25) flight_data_body_coor(:,8)]; % phi T vx_body
% y = m*flight_data_body_coor(:,5);  % m * ax
% x_axis_body_fit_fun = @(beta,x)(-m*g*sin(x(:,1))+x(:,2)*beta.*x(:,3));
% beta0 = -0.05;
% 
% beta_estimate = nlinfit(x,y,x_axis_body_fit_fun,beta0);
% beta_estimate1 = lsqcurvefit(x_axis_body_fit_fun,beta0,x,y);

for i = 1:5:size(flight_data_body_coor,1)
plot(flight_data_body_coor(i,8),flight_data_body_coor(i,5),'o');
hold on;
end
% % body y axis
% x = [flight_data_body_coor(:,2) flight_data_body_coor(:,3) flight_data_body_coor(:,25) flight_data_body_coor(:,9)];
% y = m*flight_data_body_coor(:,6);
% y_axis_body_fit_fun = @(beta,x)(m*g*sin(x(:,1)).*sin(x(:,2))+x(:,3)*beta.*x(:,4));
% beta0 = -0.05;
% 
% beta_estimate = nlinfit(x,y,y_axis_body_fit_fun,beta0);
% beta_estimate1 = lsqcurvefit(y_axis_body_fit_fun,beta0,x,y);

% body x axis
x1 = [flight_data_body_coor(:,25) flight_data_body_coor(:,8)];
y1 = flight_data_body_coor(:,5);
%x_axis_body_fit_fun = @(beta,x)(-x(:,1)/m*beta.*x(:,2));
x_axis_body_fit_fun = @(beta,x)(-g*beta.*x1(:,2));
beta0 = -0.05;
beta_estimate1 = nlinfit(x1,y1,x_axis_body_fit_fun,beta0);
beta_estimate2 = lsqcurvefit(x_axis_body_fit_fun,beta0,x1,y1);

x2 = [flight_data_body_coor(:,25) flight_data_body_coor(:,9)];
y2 = flight_data_body_coor(:,6);
%y_axis_body_fit_fun = @(beta,x)(-x(:,1)/m*beta.*x(:,2));
y_axis_body_fit_fun = @(beta,x)(-g*beta.*x2(:,2));
beta0 = -0.05;
beta_estimate3 = nlinfit(x2,y2,x_axis_body_fit_fun,beta0);
beta_estimate4 = lsqcurvefit(x_axis_body_fit_fun,beta0,x2,y2);

temp = 1;