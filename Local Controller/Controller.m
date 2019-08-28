function u = Controller(x, xr , vr , wr , Kx, Ky , Kt)

    u = zeros(2,1);

    R = [cos(x(3)) , sin(x(3)) , 0;
         -sin(x(3)) , cos(x(3)) , 0;
          0 , 0 , 1];
      
    e = R * (xr - x);
    
    u(1) = vr * cos(e(3)) + Kx*e(1);
    u(2) = wr + vr * (Ky*e(2) + Kt * sin(e(3)));
end

