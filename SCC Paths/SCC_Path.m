function state = SCC_Path(path,s)

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
        if s <= Clothoid_Length(turn)
            state = Clothoid_First_State(turn,s);
        else
            state = Clothoid_Second_State(turn,s);
        end
        
    else
        
        if s <= Clothoid_Length(turn)
            state = Clothoid_First_State(turn,s);
        elseif s <= (Clothoid_Length(turn) + Circular_Arc_Length(turn))
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
        
        x = scale * fresnelc( s / scale);
        y = scale * fresnels( s / scale);
        theta = 0.5 * turn.sigma_small_delta * (s^2);
        kappa = turn.sigma_small_delta * s;
        
    else
        
        scale = sqrt(pi / turn.turn_param.sigma_max);
        
        x = scale * fresnelc( s / scale);
        y = scale * fresnels( s / scale);
        theta = 0.5 * turn.turn_param.sigma_max * s^2;
        kappa = s * Clothoid_Length(turn) / turn.turn_param.k_max; 
        
    end
    
    state = [ x ; y ; theta ; kappa];
end

function state = Clothoid_Second_State(turn,s)

    s_new = turn.length - s;
    
    if turn.uses_small_angle
        scale = sqrt(pi / turn.sigma_small_delta);
        
        x = -scale * fresnelc(s_new / scale);
        y = scale * fresnels(s_new / scale);
        theta = -0.5 * (s_new^2) * turn.sigma_small_delta;
        kappa = s_new * turn.sigma_small_delta;
        
        state = [ x ; y ; theta ; kappa];
        state = Rotate_then_Translate(state,turn.delta,turn.qg(1) , turn.qg(2) * turn.dir);
        
    else
        
        scale = sqrt( pi / turn.turn_param.sigma_max);
        
        x = -scale * fresnelc(s_new / scale);
        y = scale * fresnels(s_new / scale);
        theta = -0.5 * (s_new^2) * turn.turn_param.sigma_max;
        kappa = s_new * turn.turn_param.k_max / Clothoid_Length(turn);
        
        state = [ x ; y ; theta ; kappa];
        state = Rotate_then_Translate(state,turn.delta,turn.qg(1) , turn.qg(2) * turn.dir);
    end

end

function state = Circular_Arc_State(turn,s)

    angle_range = turn.delta - turn.turn_param.delta_min;
    start_angle = turn.turn_param.delta_min / 2;
    
    theta = start_angle + ((s - Clothoid_Length(turn))/Circular_Arc_Length(turn))*angle_range;
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
%% Path type "Macros"
function type = LSL()
    type = 1;
end

function type = LSR()
    type = 2;
end

function type = RSL()
    type = 3;
end

function type = RSR()
    type = 4;
end

%% Helper Functions

% Returns true if q1 and q2 are colinear and share the same orientation
function colinear = Colinear(q1,q2)

    equal_thetas = round(q1(3)-q2(3),2) == 0;
    
    v1 = round([cos(q1(3)),sin(q1(3))],2);
    v2 = round([q2(1)-q1(1),q2(2)-q1(2)],2);
    
    angle = round(acos(dot(v1,v2) / (norm(v2)*norm(v1))),2);
    
    colinear = equal_thetas && (angle == 0);
end

% Returns euclidean distance between configurations
function d = dist(q1,q2)

    d = norm(q2(1:2) - q1(1:2));

end
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

%% CC Turn Functions

% Returns Turn parameters given input angle
function turn = Turn( angle , turn_param )

    turn.delta = abs(angle);
    turn.dir = sign(angle);
    turn.turn_param  = turn_param;
    
    turn.uses_small_angle = turn.delta < turn_param.delta_min;
    
    turn.sigma_small_delta = Small_Sigma(turn);
    
    % turn length 
    if turn.uses_small_angle
        turn.length = 2*Clothoid_Length(turn);
    else
        turn.length = 2*Clothoid_Length(turn) + Circular_Arc_Length(turn);
    end
    
    turn.qi = state_qi(turn);
    turn.qj = state_qj(turn);
    turn.qg = state_qg(turn);


end

% Returns curvature derivative for small angles
function sigma = Small_Sigma(turn)

    Cf = fresnelc(sqrt(turn.delta/pi));
    Sf = fresnels(sqrt(turn.delta/pi));
    
    RT = turn.turn_param.outer_rad;
    gamma = turn.turn_param.gamma;
    
    sigma = (pi*(cos(turn.delta/2)*Cf + sin(turn.delta/2)*Sf)^2)/(RT*sin(turn.delta/2 + gamma))^2;
    
end

% Returns Clothoid length
function L = Clothoid_Length(turn)

    if turn.uses_small_angle
        L = sqrt( turn.delta / turn.sigma_small_delta );
    else
        L = turn.turn_param.k_max / turn.turn_param.sigma_max;
    end

end

% Returns Circular Arc Length
function L = Circular_Arc_Length(turn)
    L = (turn.delta - turn.turn_param.delta_min)/turn.turn_param.k_max;
end

% Returns intermediate config between first clothoid and circular arc
function qi = state_qi(turn)

    state = turn.turn_param.qi;
    qi = [state(1) ; turn.dir * state(2) ; turn.dir *state(3) ; turn.dir *state(4)];

end

% Returns intermediate config between circular arc and second clothoid
function qj = state_qj(turn)

    theta = turn.delta - turn.turn_param.delta_min/2;
    x = turn.turn_param.om_left(1) + turn.turn_param.inner_rad * sin(theta);
    y = turn.turn_param.om_left(2) + turn.turn_param.inner_rad * cos(theta);
    kappa = turn.turn_param.k_max;
    
    qj = [ x ; turn.dir * y ; turn.dir * theta ; turn.dir * kappa];
end

% Returns final configuration
function qg = state_qg(turn)

    qg = zeros(4,1);
    qg(1) = -turn.turn_param.om_left(1);
    qg(2) = -turn.turn_param.om_left(2);
    qg(3) = turn.delta;
    
    qg = Rotate_then_Translate( qg ,turn.delta + 2 * turn.turn_param.gamma, turn.turn_param.om_left(1), turn.turn_param.om_left(2) );
    
    qg(2) = turn.dir * qg(2);
    qg(3) = turn.dir * turn.delta;
    qg(4) = turn.dir * 0;
end
