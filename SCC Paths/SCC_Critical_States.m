function states = SCC_Critical_States(path)

    if path.type == STRAIGHT
        
        states = [ path.qi , path.qf ];
        
    else
        
        % Critcal configurations between first turn and straight segment
        if path.path_param.turn_1.uses_small_angle
            
            s1 = 0;
            s2 = path.path_param.turn_1.clothoid_len;
            s3 = path.path_param.turn_1.length;
            
            q11 = path.qi;
            q12 = Rotate_then_Translate(path.path_param.turn_1.qi, path.qi(3) , path.qi(1) , path.qi(2));
            q13 = Rotate_then_Translate(path.path_param.turn_1.qg, path.qi(3) , path.qi(1) , path.qi(2));
            
            turn_1_pts = [[q11 ; s1] ,[q12 ; s2], [q13 ; s3]];
        else
            q11 = path.qi;
            q12 = Rotate_then_Translate(path.path_param.turn_1.qi, path.qi(3) , path.qi(1) , path.qi(2));
            q13 = Rotate_then_Translate(path.path_param.turn_1.qj, path.qi(3) , path.qi(1) , path.qi(2));
            q14 = Rotate_then_Translate(path.path_param.turn_1.qg, path.qi(3) , path.qi(1) , path.qi(2));
            
            s1 = 0;
            s2 = path.path_param.turn_1.clothoid_len;
            s3 = path.path_param.turn_1.arc_len + s2;
            s4 = path.path_param.turn_1.length;
            
            turn_1_pts = [[q11 ; s1] , [q12 ; s2] , [q13 ; s3], [q14 ; s4]];
        end
        
        % Critical points between straight segment and second turn
        
        S = path.path_param.length - path.path_param.turn_2.length;
        
        if path.path_param.turn_1.uses_small_angle
            
            s1 = S;
            s2 = S + path.path_param.turn_2.clothoid_len;
            s3 = path.path_param.length;
            
            q21 = path.path_param.q2;
            q21(3) = path.path_param.q1(3);
            q22 = Rotate_then_Translate(path.path_param.turn_2.qi , path.qf(3) + pi , path.qf(1) , path.qf(2));
            q22(3) = mod(q22(3) - path.qf(3) - pi + path.path_param.q12_angle - path.path_param.turn_2_angle,2*pi);
            q23 = path.qf;
            
            turn_2_pts = [[q21 ; s1] ,[q22 ; s2] ,[q23 ; s3]];
        else
            
            s1 = S;
            s2 = S + path.path_param.turn_2.clothoid_len;
            s3 = s2 + path.path_param.turn_2.arc_len;
            s4 = path.path_param.length;
            
            q21 = path.path_param.q2;
            q21(3) = path.path_param.q1(3);
            q22 = Rotate_then_Translate(path.path_param.turn_2.qj , path.qf(3) + pi , path.qf(1) , path.qf(2));
            q22(3) = mod(q22(3) - path.qf(3) - pi + path.path_param.q12_angle - path.path_param.turn_2_angle,2*pi);
            q23 = Rotate_then_Translate(path.path_param.turn_2.qi , path.qf(3) + pi , path.qf(1) , path.qf(2));
            q23(3) = mod(q23(3) - path.qf(3) - pi + path.path_param.q12_angle - path.path_param.turn_2_angle,2*pi);
            q24 = path.qf;
            
            turn_2_pts = [[q21 ; s1] , [q22 ; s2], [q23 ; s3], [q24 ; s4]];
        end
        
        states = [turn_1_pts , turn_2_pts];
        
    end
end


function s = STRAIGHT()
    s = 5;
end


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
