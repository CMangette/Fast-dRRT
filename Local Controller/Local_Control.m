function u_t = Local_Control(x,t,v_t,path)
    u_t = zeros(2,1);
    kappa_ref = PurePursuitController(x,path);
    v_ref = interp1(v_t(1,:),v_t(2,:),t);
    
    u_t = [kappa_ref ; v_ref];
end

