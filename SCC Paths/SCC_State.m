function state = SCC_State(path,s)

    state = zeros(4,length(s));
    
    for i = 1:length(s)
        
        if s(i) <= path.path_param.turn_1.length
            
            state(:,i) = Turn_State(path.path_param.turn_1,s(i));
            
            state(:,i) = Rotate_then_Translate(state(:,i), path.qi(3) , path.qi(1), path.qi(2));
        elseif s(i) <= path.path_param.length - path.path_param.turn_2.length
            
            state(:,i) = Segment_State(path,s(i));
            
        else
            s_new = path.path_param.length - s(i);
            state(:,i) = Turn_State(path.path_param.turn_2,s_new);
            state(:,i) = Rotate_then_Translate(state(:,i), path.qf(3) + pi,path.qf(1),path.qf(2));
            state(3,i) = mod(state(3,i) - path.qf(3) - pi + path.path_param.q12_angle - path.path_param.turn_2_angle,2*pi);
        end
        
    end

end

% Returns a single turn state
function state = Turn_State(turn,s)

    if turn.uses_small_angle
        
        % If using small delta angle, choose between one of 
        if s <= turn.clothoid_len
            state = Clothoid_First_State(turn,s);
        else
            state = Clothoid_Second_State(turn,s);
        end
        
    else
        
        if s <= turn.clothoid_len
            state = Clothoid_First_State(turn,s);
        elseif s <= (turn.clothoid_len + turn.arc_len)
            state = Circular_Arc_State(turn,s);
        else
            state = Clothoid_Second_State(turn,s);
        end
        
    end
    
    state(2) = turn.dir * state(2);
    state(3) = turn.dir * state(3);
    state(4) = turn.dir * state(4);

end

function state = Clothoid_First_State(turn,s)

    if turn.uses_small_angle
        
        scale = sqrt(pi / turn.sigma_small_delta);
        [C,S] = fcs(s / scale);
        x = scale * C;
        y = scale * S;
        theta = 0.5 * turn.sigma_small_delta * (s^2);
        kappa = turn.sigma_small_delta * s;
        
    else
        
        scale = sqrt(pi / turn.turn_param.sigma_max);
        [C,S] = fcs(s/scale);
        x = scale * C;
        y = scale * S;
        theta = 0.5 * turn.turn_param.sigma_max * s^2;
        kappa = s * turn.clothoid_len / turn.turn_param.k_max; 
        
    end
    
    state = [ x ; y ; theta ; kappa];
end

function state = Clothoid_Second_State(turn,s)

    s_new = turn.length - s;
    
    if turn.uses_small_angle
        scale = sqrt(pi / turn.sigma_small_delta);
        
        [C,S] = fcs(s_new / scale);
        x = -scale * C;
        y = scale * S;
        theta = -0.5 * (s_new^2) * turn.sigma_small_delta;
        kappa = s_new * turn.sigma_small_delta;
        
        state = [ x ; y ; theta ; kappa];
        state = Rotate_then_Translate(state,turn.delta,turn.qg(1) , turn.qg(2) * turn.dir);
        
    else
        
        scale = sqrt( pi / turn.turn_param.sigma_max);
        [C,S] = fcs(s_new/scale);
        x = -scale * C;
        y = scale * S;
        theta = -0.5 * (s_new^2) * turn.turn_param.sigma_max;
        kappa = s_new * turn.turn_param.k_max / turn.clothoid_len;
        
        state = [ x ; y ; theta ; kappa];
        state = Rotate_then_Translate(state,turn.delta,turn.qg(1) , turn.qg(2) * turn.dir);
    end

end

function state = Circular_Arc_State(turn,s)

    angle_range = turn.delta - turn.turn_param.delta_min;
    start_angle = turn.turn_param.delta_min / 2;
    
    theta = start_angle + ((s - turn.clothoid_len)/turn.arc_len)*angle_range;
    x = turn.turn_param.om_left(1) + turn.turn_param.inner_rad * sin(theta);
    y = turn.turn_param.om_left(2) - turn.turn_param.inner_rad * cos(theta);
    kappa = turn.turn_param.k_max;
    
    state = [x ; y ; theta ; kappa];

end

function state = Segment_State(path,s)

    s_new = s - path.path_param.turn_1.length;
    
    xi = path.path_param.q1(1); yi = path.path_param.q1(2);
    tx = cos(path.path_param.q1(3)) ; ty = sin(path.path_param.q1(3));
    
    x = xi + tx*s_new;
    y = yi + ty*s_new;
    theta = path.path_param.q1(3);
    kappa = 0;
    
    state = [x;y;theta;kappa];

end
%% Helper Functions

% Returns the input configuration, rotated by ( theta ) degrees, and
% translated by [dx,dy] units
function q_new = Rotate_then_Translate(q,alpha,dx,dy)

    % q - (4 x 1) state vector
    % alpha - rotation angle (radians)
    % dx,dy - translation components

    R = [cos(alpha) , -sin(alpha);
         sin(alpha) , cos(alpha)];
    
    q_new = zeros(size(q));
    q_new(1:2) = R*q(1:2) + [dx;dy];
    q_new(3) = mod(alpha + q(3), 2*pi);
    q_new(4) = q(4);
end