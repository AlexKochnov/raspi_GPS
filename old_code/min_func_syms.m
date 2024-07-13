% syms x y z t
% syms xi yi zi c rhoi
% syms func(x, y, z, t)
% syms rho(x, y, z)
% syms drho(x, y, z, t)
% 
% rho(x, y, z) = sqrt((x-xi)^2 + (y-yi)^2 + (z-zi)^2) ;
% drho(x, y, z, t) = rho(x, y, z) + c*t - rhoi;
% % func(x, y, z, t) = (rho(x, y, z) + c * t - rhoi) ^ 2s
% func = drho(x, y, z, t) ^ 2
% 
% disp(func)
% 
% jac = jacobian(func, [x, y, z, t])
% 
% hess = hessian(func, [x, y, z, t])
% 
% 
% % for i = 1:length(jac)
%     % ji = jac(i)
% % end


syms DRHO RHO2 D2X D2Y D2Z
syms hess(DRHO, RHO2, D2X, D2Y, D2Z)

% hess = sym('a', zeros(4, 4))

% hess1 = [
%     (2*DRHO)/RHO2^(1/2) + D2X^2/(2*RHO2) - (D2X^2*DRHO)/(2*RHO2^(3/2))
% (D2X*D2Y)/(2*RHO2) - (D2X*D2Y*DRHO)/(2*RHO2^(3/2))
% (D2X*D2Z)/(2*RHO2) - (D2X*D2Z*DRHO)/(2*RHO2^(3/2))
% (c*D2X)/RHO2^(1/2)
% ];
% 
% hess2 = [
% (D2X*D2Y)/(2*RHO2) - (D2X*D2Y*DRHO)/(2*RHO2^(3/2))
% (2*DRHO)/RHO2^(1/2) + D2Y^2/(2*RHO2) - (D2Y^2*DRHO)/(2*RHO2^(3/2))
% (D2Y*D2Z)/(2*RHO2) - (D2Y*D2Z*DRHO)/(2*RHO2^(3/2))
% (c*D2Y)/RHO2^(1/2);
% ];



hess = [

transpose([
(2*DRHO)/RHO2^(1/2) + D2X^2/(2*RHO2) - (D2X^2*DRHO)/(2*RHO2^(3/2)) 
(D2X*D2Y)/(2*RHO2) - (D2X*D2Y*DRHO)/(2*RHO2^(3/2))
(D2X*D2Z)/(2*RHO2) - (D2X*D2Z*DRHO)/(2*RHO2^(3/2))
(c*D2X)/RHO2^(1/2)
]);

transpose([
(D2X*D2Y)/(2*RHO2) - (D2X*D2Y*DRHO)/(2*RHO2^(3/2))
(2*DRHO)/RHO2^(1/2) + D2Y^2/(2*RHO2) - (D2Y^2*DRHO)/(2*RHO2^(3/2))
(D2Y*D2Z)/(2*RHO2) - (D2Y*D2Z*DRHO)/(2*RHO2^(3/2))
(c*D2Y)/RHO2^(1/2)
]);

transpose([
(D2X*D2Z)/(2*RHO2) - (D2X*D2Z*DRHO)/(2*RHO2^(3/2))
(D2Y*D2Z)/(2*RHO2) - (D2Y*D2Z*DRHO)/(2*RHO2^(3/2))
(2*DRHO)/RHO2^(1/2) + D2Z^2/(2*RHO2) - (D2Z^2*DRHO)/(2*RHO2^(3/2))
(c*D2Z)/RHO2^(1/2)
]);

transpose([
(c*D2X)/RHO2^(1/2) 
(c*D2Y)/RHO2^(1/2)
(c*D2Z)/RHO2^(1/2)
2*c^2
]);

]
