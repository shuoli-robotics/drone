clear
clc

beta = [5 6 7 8 9];


modelfun  = @(beta,x)(beta(1)*x(:,2)-x(:,3)/beta(5))./(1+beta(2)*x(:,1)+beta(3)*x(:,2)+beta(4)*x(:,3));

x = 10*rand(100,4);
y = modelfun(beta,x);  


 
beta0 = 15*[5 5 5 5 5];
beta_estimate = nlinfit(x,y,modelfun,beta0);
beta_estimate1 = lsqcurvefit(modelfun,beta0,x,y);
temp = 1;