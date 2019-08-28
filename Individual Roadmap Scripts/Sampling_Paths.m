function ref_paths = Sampling_Paths(qi,qf,intention,N)
%SAMPLING_PATHS Returns the ideal paths to sample along given qi, qf, and
%intention
%
% qi - (3 x 1) initial configuration
% qf - (3 x 1) final configuration
    
    ref_paths = cell(1,2);

    switch intention
        
        case STRAIGHT
            
            % Endpoints of first path
            q1 = qi;
            q2 = qf;
            
            ref_paths{1} = Linear_Segment(q1,q2,N);

            % Endpoints of second path
            %q3 = Shift_Config(q1,0,-4);
            %q4 = Shift_Config(q2,0,-4);
            
            %ref_paths{2} = Linear_Segment(q3,q4,N);
            
        case RIGHT
            
            % Endpoints of first path
            q1 = qi;
            q2 = qf;
            
            ref_paths{1} = Curve(q1,q2,N,RIGHT);
            
            %q3 = Shift_Config(q1,0,4);
            %q4 = Shift_Config(q2,0,4);
            
            %ref_paths{2} = Curve(q3,q4,N,RIGHT);
            
        case LEFT
            
            q1 = qi;
            q2 = qf;
            
            ref_paths{1} = Curve(q1,q2,N,LEFT);
            
            %q3 = Shift_Config(q1,0,-4);
            %q4 = Shift_Config(q2,0,-4);
            
            %ref_paths{2} = Curve(q3,q4,N,LEFT);
            
    end

end

%% Helper Functions

% Translates origingal configuration [dx,dy] units relative to local
% reference frame
function q = Shift_Config(q,dx,dy)

    R = [ cos(q(3)) , -sin(q(3)) ; sin(q(3)) , cos(q(3)) ];
    
    q(1:2) = q(1:2) + R * [dx ; dy];

end

% Returns N sample points along a linear path characterized by starting
% config qi and end config qf
function pts = Linear_Segment(qi,qf,N)

    % overall path length
    L = dist(qi,qf);
    
    
    x = linspace(qi(1),qf(1),N);
    y = linspace(qi(2),qf(2),N);
    theta = qi(3)*ones(1,N);
    s = linspace(0,L,N);
    
    pts = [ x ; y ; theta ; s];
    

end

% Returns euclidean distance between configurations
function d = dist(q1,q2)
    d = norm(q2(1:2) - q1(1:2));
end

% Returns a spline curve characterized by its starting configuration and end configuration
function pts = Curve(qi,qf,N,dir)

    % Find intermediate point between initial and final configuration
    q_j = Intersection_Point(qi,qf);
    
    q1 = Shift_Config([q_j;qi(3)],-4,0);
    q2 = Shift_Config([q_j;qf(3)],4,0);
    
    
    
    first_segment = Linear_Segment(qi,q1,round(N/3,0));
    
    circular_arc = Arc_Segment(q1,q2,4,round(N/3,0),dir);
    circular_arc(4,:) = circular_arc(4,:) + first_segment(4,end);
    
    second_segment = Linear_Segment(q2,qf,round(N/3,0));
    second_segment(4,:) = second_segment(4,:) + circular_arc(4,end);
    
    pts = [ first_segment(:,1:(end-1)) , circular_arc(:,(1:end-1)) , second_segment ];
end

function pts = Intersection_Point(xi,xf)

    Ti = [cos(xi(3));sin(xi(3))];
    Tf = [cos(xf(3));sin(xf(3))];
    
    pts = zeros(2,1);
    if(round(Ti(1),3) == 0)
        pts(1) = xi(1);
        pts(2) = xf(2);
    else
        pts(1) = xf(1);
        pts(2) = xi(2);
    end

end


function pts = Arc_Segment(q1,q2,R,N,dir)

    if dir == LEFT
        
        circle_center = Shift_Config(q1,0,4);
        
    else
        
        circle_center = Shift_Config(q1,0,-4);
        
    end
    
    circle_center(3) = [];
    
    direction = sign(dir - 2);
    % Path length
    
    if(direction == 1)
        angle_range = pi/2;
    else
        angle_range = -pi/2;
    end
    
    L = R * abs(angle_range);
    
    xc = circle_center(1);
    yc = circle_center(2);
    
    s = linspace(0,L,N);
    
    theta = q1(3) + s .* angle_range / L;
    x = xc + R * direction * sin(theta);
    y = yc - R * direction * cos(theta);
    
    pts = [x ; y ; theta ; s];

end
%% "Macro" definitions
function r = RIGHT()
    r = 1;
end

function s = STRAIGHT()
    s = 2;
end

function l = LEFT()
    l = 3;
end
