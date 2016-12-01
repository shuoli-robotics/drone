function [ y ] = myfitfun( beta,x )
%MYFITFUN 此处显示有关此函数的摘要
%   此处显示详细说明

y = (beta(1)*x(2)-x(3)/beta(5))/(1+beta(2)*x(1)+beta(3)*x(2)+beta(4)*x(3));

end

