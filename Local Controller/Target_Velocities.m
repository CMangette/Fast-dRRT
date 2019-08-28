function [v_t, omega_t , x_t] = Target_Velocities(wp , v_optimal , dt)
    
    % Choosing an initial speed. If the first time interval requirest the
    % vehicle to stop, then vi = 0. Otherwise, choose v_opt.
    if isempty(wp(1).ds)
        v_initial = 0; 
    else 
        v_initial = v_optimal;
    end
    
    
    T = 0;
    S = 0;
    S_local = 0;
    x_s = [];
    s_t = [];
    v_t = [];
    num_wp = length(wp);
    for i = 1:(num_wp - 1)
        
        Ti = wp(i).state(5); % Time at initial waypoint
        Tf = wp(i+1).state(5); % Time at final waypoint
        
        
        
        % Case 1: hold position
        if isempty(wp(i).ds)
            t = Ti:dt:Tf; % Time interval between waypoints
            s = S + zeros(size(t));
            v = zeros(size(t));
            
            v_t = [ v_t , [t(1:end-1) ; v(1:end-1)]];
            s_t = [s_t , [t ; s]];
            
            T = t(end);
        else
            
            t = 0:dt:(Tf - Ti); % relative time interval between waypoints
            T1 = 0.125 * (Tf - Ti); % Accelerating / Decelerating interval
            T2 = 0.875 * (Tf - Ti); % Coasting interval
            
            % Case 2: move forward, but next interval requires holding position
            if isempty(wp(i+1).ds)
                
                v_final = 0;
            
            % Case 3: move forward
            else
                
                v_final = v_optimal;
                
            end
            
            v_coasting = coasting_velocity(wp(i).ds , v_initial , v_final);
            
            % First time interval
            t_1 = t( t <= T1 );
            a_1 = (v_coasting - v_initial)/T1;
            v_1 = linspace(v_initial,v_coasting,length(t_1));
            
            % Second time interval
            t_2 = t((t > T1) & (t <= T2));
            a_2 = 0;
            v_2 = v_coasting * ones(size(t_2));
            
            % Third time interval
            t_3 = t(t > T2);
            v_3 = linspace(v_coasting,v_final,length(t_3));
            
            v = [v_1 , v_2 , v_3];
            t = T + t;
            
            v_t = [v_t , [t(1:end-1) ; v(1:end-1)]];
            
            local_path = wp(i).ref_path(:,1:end);
            local_path(5,:) = S_local + local_path(5,:);
            
            x_s = [x_s , local_path];
            
            S_local = S_local + wp(i).ds;
            T = t(end);
        end
        
        v_initial = v_final;
    end
    
    s_t = Integrate_Velocity(v_t , dt);
    [~,unique_indices] = unique(x_s(5,:));
    x_s = x_s(:,unique_indices);
    x_t = zeros(4,size(s_t,2));
    
    x_t(1,:) = v_t(1,:);
    x_t(2,:) = interp1(x_s(5,:),x_s(1,:),s_t);
    x_t(3,:) = interp1(x_s(5,:),x_s(2,:),s_t);
    x_t(4,:) = interp1(x_s(5,:),x_s(3,:),s_t);
    kappa_t = interp1(x_s(5,:),x_s(4,:),s_t);
    omega_t = [v_t(1,:) ;
               v_t(2,:) .* kappa_t];
end

% Returns coasting velocity 
function v_coast = coasting_velocity(ds,vi,vf)
    v_coast = (ds - 0.0625 * vf - 0.0625 * vi)/0.875;
end

function s = Integrate_Velocity(v_t,dt)
    
    s = zeros(1,size(v_t,2));
    for i = 2:size(v_t,2)
       s(i) = s(i - 1) + dt * v_t(2,i-1); 
    end

end

