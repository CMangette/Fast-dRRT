function path_polygon = Path_Polygon(path,vehicle_vertices)

    if path.type == STRAIGHT
        
        path_polygon = Segment_Polygon(path.qi , path.qf, vehicle_vertices);
        
    else
        
        turn1_poly = Turn1_Polygon(path, path.path_param.turn_1 , vehicle_vertices);
        
        q1 = path.path_param.q1;
        q2 = path.path_param.q2;
        q2(3) = q1(3);
        segment_poly = Segment_Polygon(q1,q2,vehicle_vertices); 
        turn2_poly = Turn2_Polygon(path, path.path_param.turn_2 , vehicle_vertices);
        
        path_polygon = union( turn1_poly , union(segment_poly , turn2_poly));
    end

end

%% Line Segment Polygon
function poly_out = Segment_Polygon(q1 , q2, vehicle_vertices)

    rear_pts = Convert_to_Ref_Frame(q1,vehicle_vertices(:,1:2));
    front_pts = Convert_to_Ref_Frame(q2,vehicle_vertices(:,3:4));
    
    polygon_pts = [rear_pts , front_pts];
    
    poly_out = polyshape(polygon_pts(1,:),polygon_pts(2,:));

end

%% Turning Polygon Functions

function poly_out = Turn1_Polygon(path ,turn,vehicle_vertices)

    if turn.uses_small_angle
        
        L1 = turn.clothoid_len;
        L2 = 2 * turn.clothoid_len;
        
        s = [ 0 , L1 , L2];
        
        q = SCC_Path(path,s);
        
        if path.type == LSL || path.type == LSR
            dir = LEFT;
        else
            dir = RIGHT;
        end
        
        sub_poly_1 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        sub_poly_2 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        
        poly_out = union(sub_poly_1 , sub_poly_2);
    else
        
        L1 = path.path_param.turn_1.clothoid_len;
        L2 = L1 + path.path_param.turn_1.arc_len;
        L3 = L1 + L2;
        
        s = [ 0 , L1 , L2 , L3];
        
        q = SCC_Path(path,s);
        
        
        if path.type == LSL || path.type == LSR
            dir = LEFT;
        else
            dir = RIGHT;
        end
        
        sub_poly_1 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        sub_poly_2 = Arc_Poly(L1 , L2 , path, vehicle_vertices);
        sub_poly_3 = Clothoid_Poly(q(:,3) , q(:,4) , vehicle_vertices , dir);
        
        poly_out = union(sub_poly_1 , union(sub_poly_2 , sub_poly_3));
    end

end


function poly_out = Turn2_Polygon(path, turn, vehicle_vertices)

    if turn.uses_small_angle
        
        L1 = path.path_param.length - turn.length;
        L2 = L1 + turn.clothoid_len;
        L3 = path.path_param.length;
        
        s = [ L1 , L2 , L3];
        
        q = SCC_Path(path,s);
        
        if path.type == LSL || path.type == RSL
            dir = LEFT;
        else
            dir = RIGHT;
        end
        
        sub_poly_1 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        sub_poly_2 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        
        poly_out = union(sub_poly_1 , sub_poly_2);
    else
        
        L1 = path.path_param.length - turn.length;
        L2 = L1 + turn.clothoid_len;
        L3 = L2 + turn.arc_len;
        L4 = path.path_param.length;
        
        s = [ L1 , L2 , L3 , L4];
        
        q = SCC_Path(path,s);
        
        
        if path.type == LSL || path.type == RSL
            dir = LEFT;
        else
            dir = RIGHT;
        end
        
        sub_poly_1 = Clothoid_Poly(q(:,1) , q(:,2) , vehicle_vertices , dir);
        sub_poly_2 = Arc_Poly(L2 , L3 , path , vehicle_vertices);
        sub_poly_3 = Clothoid_Poly(q(:,3) , q(:,4) , vehicle_vertices , dir);
        
        poly_out = union(sub_poly_1 , union(sub_poly_2 , sub_poly_3));
    end
end

function poly_out = Clothoid_Poly(q1,q2,vertices,dir)

    LR = Convert_to_Ref_Frame(q1,vertices(:,1));
    RR = Convert_to_Ref_Frame(q1,vertices(:,2));
    
    RF = Convert_to_Ref_Frame(q2,vertices(:,3));
    LF = Convert_to_Ref_Frame(q2,vertices(:,4));
    
    if dir == LEFT
        
        IP = Intersection_Point([RR ; q1(3)] , [RF ; q2(3)]);
        pts = [LR , RR, IP , RF , LF];
    else
        
        IP = Intersection_Point([LR ; q1(3)], [LF ; q2(3)]);
        pts = [ LR , RR , RF , LF, IP];
    end
    
    poly_out = polyshape(pts(1,:),pts(2,:));

end


function poly_out = Arc_Poly(L1,L2,path,vehicle_vertices)

    s = linspace(L1,L2,5);
    q = SCC_Path(path,s);
    
    poly_out = polybuffer(q(1:2,:)','lines',0.7);

%     vehicle_left_pts = zeros(2,6);
%     vehicle_right_pts = zeros(2,6);
%     for i = 1:length(s)
%         
%         vehicle_left_pts(:,i) = Convert_to_Ref_Frame(q(:,i),vehicle_vertices(:,1));
%         vehicle_right_pts(:,i) = Convert_to_Ref_Frame(q(:,i),vehicle_vertices(:,2));
%         
%     end
%     
%     vehicle_left_pts(:,6) = Convert_to_Ref_Frame(q(:,5),vehicle_vertices(:,4));
%     vehicle_right_pts(:,6) = Convert_to_Ref_Frame(q(:,5),vehicle_vertices(:,3));
%     
%     vehicle_right_pts = flip(vehicle_right_pts,2);
%     vehicle_pts = [vehicle_left_pts , vehicle_right_pts];
    
%     poly_out = polyshape(vehicle_pts(1,:),vehicle_pts(2,:));

end
%% Miscellaneous Functions

function q = Convert_to_Ref_Frame(qref,q)

    theta = qref(3);
    R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];

    % Case 1: q is an entire configuration 
    if(size(q,1) == 4)
 
        for i = 1:size(q,2)
            dq = q(1:2,i);
            q(1:2,i) = R*dq + qref(1:2);
            q(3,i) = mod(q(3,i) + theta,2*pi);
        end
    % Case 2: q is a set of euclidean points
    else
        for i = 1:size(q,2)
            dq = q(1:2,i);
            q(1:2,i) = R*dq + qref(1:2);
        end
    end
end


% Returns the input configuration, rotated by ( theta ) degrees, and
% translated by [dx,dy] units

function pts = Intersection_Point(q1,q2)

    ax = q1(1); ay = q1(2); tx_a = cos(q1(3)); ty_a = sin(q1(3));
    bx = q2(1); by = q2(2); tx_b = cos(q2(3) + pi); ty_b = sin(q2(3) + pi);
    
    lhs  = (ay - by) + ty_a*(bx - ax)/tx_a ;
    rhs = ty_b - (ty_a*tx_b/tx_a);
    
    v = lhs / rhs;
    
    u = ((bx - ax) + tx_b*v)/tx_a;
    
    pts = zeros(2,1);
    pts(1) = ax + tx_a*u;
    pts(2) = ay + ty_a*u;

end
%% "Macro" definitions

function type = STRAIGHT()
    type = 5;
end

function l = LEFT()
    l = 1;
end

function r = RIGHT()
    r = -1;
end

function type = LSL()
    type = 1;
end

function type = LSR()
    type = 2;
end

function type = RSL()
    type = 3;
end

