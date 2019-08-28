function f = Fx(x,u)
%FX State transition function
%
% x = [ px , py , theta , kappa , v , s]'
%
% px'(t) = v(t)cos(theta(t))
% py'(t) = v(t)sin(theta(t))
% theta'(t) = kappa(t)
% kappa'(t) = u1
% v'(t) = u2
% s'(t) = v(t)

f = zeros(6,1);
f(1) = x(5)*cos(x(3));
f(2) = x(5)*sin(x(3));
f(3) = x(4);
f(4) = u(1);
f(5) = u(2);
f(6) = x(5);
end

