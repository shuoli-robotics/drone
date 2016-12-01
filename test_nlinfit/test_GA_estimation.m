clear
clc



%beta0 = [5 6 7 8 9];



beta = [1 1 1 1 1];

y_est = nolin_fun(beta);

ObjectiveFunction = @nolin_fun;
nvars = 5;    % Number of variables
LB = [0 0 0 0 0];   % Lower bound
UB = 20*[1 1 1 1 1];  % Upper bound
%ConstraintFunction = @simple_constraint;
[x,fval] = ga(ObjectiveFunction,nvars,[],[],[],[],LB,UB)