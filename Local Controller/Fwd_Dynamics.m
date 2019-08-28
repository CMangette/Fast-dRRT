function x = Fwd_Dynamics(x,u,dt)

    fx = zeros(5,1);
    fx(1) = x(5)*cos(x(3));
    fx(2) = x(5)*sin(x(3));
    fx(3) = x(4);
    fx(4) = u(1);
    fx(5) = u(2);
    
    x = x + dt*fx;

end

