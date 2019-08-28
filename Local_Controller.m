function u = Local_Controller(x,t,dt,x_r,v_ref)
% LOCAL_CONTROLLER Returns control vector u = [ a , alpha]'
%
% Inputs:
% x (5 x 1) : Current vehicle state
% path: (5 x N): Path X(s) states parameterized as function of arc length
% v_ref: (1 X M): Reference velocity profile v(t) for vehicle to follow.
% Parameterized in time
%
% Outputs: 
% u ( 2 x 1): Control vector


%% Choosing an acceleration signal to apply to system
    vi = x(5);
    vf = interp1(v_ref(1,:),v_ref(2,:),t + dt);
    a = (vf - vi)/dt;

%% Choosing alpha value
    
    % Calculate the distancce traveled over one time step
    si = x(6);
    sf = si + vi*dt + 0.5*a*dt^2;
    
    % Interpolate the desired final reference state from final traveled
    % distance
    x_r = interp1(path(5,:),path(1,:),sf);
    y_r = interp1(path(5,:),path(2,:),sf);
    theta_r = interp1(path(5,:),path(3,:),sf);
    
    % Desired final state
    x_f = [x_r; y_r ; theta_r];
    
    alpha = Shooting_Method(x_f,a,alpha_min,alpha_max,dt);
    

%% Output

    u = [a ; alpha];
    
end