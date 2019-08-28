function optimal_path = Steer(qi,qf,turn_param)

    if Colinear(qi,qf) 
        
        % Special case: q1 to q2 is a straight line segment 
        len = dist(qi,qf);
        
        path_param = struct('length',len);
        optimal_path = struct('qi',qi,'qf',qf,'turn_param',[],'type',STRAIGHT,'path_param',path_param,'is_valid',true);

    else

        PATH_OPTIONS = [LSL , LSR , RSL , RSR];
        
        min_path_length = Inf;
        min_path_idx = 0;
        
        for i = 1:length(PATH_OPTIONS)
            
            path(i) = SCC_Path(qi,qf,turn_param,PATH_OPTIONS(i));
            
            if(path(i).path_param.length < min_path_length)
                min_path_idx = i;
                min_path_length = path(i).path_param.length;
            end
            
        end
        
        if min_path_idx == 0
            
            optimal_path = struct('length',Inf,'pts',[],'is_valid',false);
            return;
            
        end
        optimal_path = path(min_path_idx);
        optimal_path.is_valid = true;
    end

end

%% SCC Path functions

function path  = SCC_Path(qi,qf,turn_param,type)

    path.qi = qi;
    path.qf = qf;
    path.turn_param = turn_param;
    path.type = type;
    path.path_param = Path_Param(path);
end

function param  = Path_Param(path)

    [om_1,om_2] = Omega(path);
    
    % Distance and angle between departure circles
    om12_distance = dist(om_1 , om_2);
    
    if  2 * path.turn_param.outer_rad > om12_distance
        param = struct('length',Inf);
        return;
    end
    om12_angle = atan2(om_2(2) - om_1(2) , om_2(1) - om_1(1));
    
    [turn_1_angle, turn_2_angle ,q12_angle] = Turn_Angles(path,om12_distance,om12_angle);
    
    turn_1 = Turn(turn_1_angle,path.turn_param);
    turn_2 = Turn(turn_2_angle,path.turn_param);
    
    q1 = Rotate_then_Translate(turn_1.qg,path.qi(3),path.qi(1),path.qi(2));
    q2 = Rotate_then_Translate(turn_2.qg, path.qf(3) + pi, path.qf(1) , path.qf(2));
    
    segment_length = dist(q1,q2);
    
    length = turn_1.length + segment_length + turn_2.length;
    
    param = struct('turn_1',turn_1,'turn_2',turn_2,'q1',q1,'q2',q2,'length',length,...
                    'turn_1_angle',turn_1_angle,'turn_2_angle',turn_2_angle,...
                    'q12_angle',q12_angle);


end


% Returns the centers of turn circles 1 and 2
function [om_1,om_2] = Omega(path)
    
    dx1 = path.qi(1) ; dy1 = path.qi(2); theta1 = path.qi(3);
    
    if (path.type == LSL) || (path.type == LSR) 
        om_1 = Rotate_then_Translate([path.turn_param.om_left ;0;0],theta1,dx1,dy1);
    else 
        om_1 = Rotate_then_Translate([path.turn_param.om_right;0;0],path.qi(3),path.qi(1),path.qi(2));
    end
    
    om_1(3:4) = [];
    
    dx2 = path.qf(1) ; dy2 = path.qf(2); theta2 = mod(path.qf(3) + pi, 2*pi);
    
    if (path.type == LSL) || (path.type == RSL)
        om_2 = Rotate_then_Translate([path.turn_param.om_right;0;0],theta2,dx2,dy2);
    else
        om_2 = Rotate_then_Translate([path.turn_param.om_left;0;0],theta2,dx2,dy2);
    end
    
    om_2(3:4) = [];

end

% Returns the turn angles for the specified SCC path type
function [turn1_angle,turn2_angle,q12_angle] = Turn_Angles(path,om12_distance,om12_angle)

    alpha_2 = asin((2*cos(path.turn_param.gamma)*path.turn_param.outer_rad)/om12_distance);
    
    switch path.type
        
        case LSL
            
            q12_angle = om12_angle;
            
            turn1_angle = mod(q12_angle - path.qi(3),2*pi);
            turn2_angle = mod(q12_angle - path.qf(3),2*pi) - 2*pi;
            
        case LSR
            
            q12_angle = om12_angle + alpha_2;
            
            turn1_angle = mod(q12_angle - path.qi(3),2*pi);
            turn2_angle = mod(q12_angle - path.qf(3),2*pi);         
            
        case RSL
            
            q12_angle = om12_angle - alpha_2;
            
            turn1_angle = mod(q12_angle - path.qi(3),2*pi) - 2*pi;
            turn2_angle = mod(q12_angle - path.qf(3),2*pi) - 2*pi;
            
        case RSR
            q12_angle = om12_angle;
            
            turn1_angle = mod(q12_angle - path.qi(3),2*pi) - 2*pi;
            turn2_angle = mod(q12_angle - path.qf(3),2*pi);
            
    end

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
    turn.clothoid_len = Clothoid_Length(turn);
    turn.arc_len = Circular_Arc_Length(turn);
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
    y = turn.turn_param.om_left(2) - turn.turn_param.inner_rad * cos(theta);
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


%% State Reconstruction functions
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

function type = STRAIGHT()
    type = 5;
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