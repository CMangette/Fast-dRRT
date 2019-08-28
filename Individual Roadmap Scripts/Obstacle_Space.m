function [X_obs,L_obst,sample_bounds] = Obstacle_Space(qi,qf,intention,Intersection_Poly)
    
%OBSTACLE_SPACE returns Obstacles bounding the sampling region

    dir = round(intention - 2,1);
    
    
    
    if(dir == 0)
        
        % Case 1: Straight Motion 
        left_boundary_points = zeros(2,4);
        
        left_boundary_points(:,1) = Shift_Configuration(qi,0,0.25);
        left_boundary_points(:,2) = Shift_Configuration(qf,0,0.25);
        left_boundary_points(:,3) = Shift_Configuration(qf,0,2);
        left_boundary_points(:,4) = Shift_Configuration(qi,0,2);
        
        X_obs(1) = polyshape(left_boundary_points(1,:),left_boundary_points(2,:));
        
        right_boundary_points = zeros(2,4);
        
        % Lane boundary
        right_boundary_points(:,1) = Shift_Configuration(qi,0,-0.5);
        right_boundary_points(:,2) = Shift_Configuration(qf,0,-0.5);
        right_boundary_points(:,3) = Shift_Configuration(qf,0,-3.5);
        right_boundary_points(:,4) = Shift_Configuration(qi,0,-3.5);
        
        L_obst = polyshape(right_boundary_points(1,:),right_boundary_points(2,:));
        % Right Road boundary
        right_boundary_points(:,1) = Shift_Configuration(qi,0,-4.25);
        right_boundary_points(:,2) = Shift_Configuration(qf,0,-4.25);
        right_boundary_points(:,3) = Shift_Configuration(qf,0,-6);
        right_boundary_points(:,4) = Shift_Configuration(qi,0,-6);
        
        X_obs(2) = polyshape(right_boundary_points(1,:),right_boundary_points(2,:));
        
        shifted_qi = Shift_Configuration(qi,0,-4);
        shifted_qf = Shift_Configuration(qf,0,-4);
        
        sample_bounds.xmin = min([shifted_qi(1) , shifted_qf(1) , qi(1), qf(1)]);
        sample_bounds.xmax = max([shifted_qi(1) , shifted_qf(1) , qi(1), qf(1)]);
        sample_bounds.ymin = min([shifted_qi(2) , shifted_qf(2) , qi(2), qf(2)]);
        sample_bounds.ymax = max([shifted_qi(2) , shifted_qf(2) , qi(2), qf(2)]);
        sample_bounds.theta_min = qi(3) - pi/10;
        sample_bounds.theta_max = qi(3) + pi/10;
    else
 
            % Case 2: Turning motion
   
            
            % Inner sampling boundary points
            
            inner_boundary_points = zeros(2,4);
            inner_boundary_points(:,1) = Shift_Configuration(qi,0,dir*0.5);
            inner_boundary_points(:,3) = Shift_Configuration(qf,0,dir*0.5);
            
            dx = abs(inner_boundary_points(1,3) - inner_boundary_points(1,1));
            dy = abs(inner_boundary_points(2,3) - inner_boundary_points(2,1));
            
            inner_boundary_points(:,2) = Shift_Configuration([inner_boundary_points(:,1);qi(3)],dx,0);
            inner_boundary_points(:,4) = Shift_Configuration([inner_boundary_points(:,1);qi(3)],0,dir*dy);
            
            X_obs(1) = polyshape(inner_boundary_points(1,:),inner_boundary_points(2,:));
            
            % Lane Boundary points
            outer_boundary_points = zeros(2,6);
            
            outer_boundary_points(:,1) = Shift_Configuration(qi,0,-0.5*dir);
            outer_boundary_points(:,3) = Shift_Configuration(qf,0,-0.5*dir);
            
            outer_boundary_points(:,4) = Shift_Configuration(qf,0,-3.5*dir);
            outer_boundary_points(:,6) = Shift_Configuration(qi,0,-3.5*dir);
            
            dx1 = abs(outer_boundary_points(1,3) - outer_boundary_points(1,1));
            
            dx2 = abs(outer_boundary_points(1,4) - outer_boundary_points(1,6));

            outer_boundary_points(:,2) = Shift_Configuration([outer_boundary_points(:,1);qi(3)],dx1,0);
            outer_boundary_points(:,5) = Shift_Configuration([outer_boundary_points(:,6);qi(3)],dx2,0);
            
            L_obst = polyshape(outer_boundary_points(1,:),outer_boundary_points(2,:));
            L_obst = subtract(L_obst,Intersection_Poly);
            
            % Outer Sampling Boundary points
            outer_boundary_points = zeros(2,6);
            
            outer_boundary_points(:,1) = Shift_Configuration(qi,0,-4.5*dir);
            outer_boundary_points(:,3) = Shift_Configuration(qf,0,-4.5*dir);
            
            outer_boundary_points(:,4) = Shift_Configuration(qf,0,-6*dir);
            outer_boundary_points(:,6) = Shift_Configuration(qi,0,-6*dir);
            
            dx1 = abs(outer_boundary_points(1,3) - outer_boundary_points(1,1));
            
            dx2 = abs(outer_boundary_points(1,4) - outer_boundary_points(1,6));

            outer_boundary_points(:,2) = Shift_Configuration([outer_boundary_points(:,1);qi(3)],dx1,0);
            outer_boundary_points(:,5) = Shift_Configuration([outer_boundary_points(:,6);qi(3)],dx2,0);
        
            X_obs(2) = polyshape(outer_boundary_points(1,:),outer_boundary_points(2,:));
            
            % Sampling bounds parameters
            shifted_qi = Shift_Configuration(qi,0,-4*dir);
            shifted_qf = Shift_Configuration(qf,0,-4*dir);
        
            sample_bounds.xmin = min([shifted_qi(1) , shifted_qf(1) , qi(1), qf(1)]);
            sample_bounds.xmax = max([shifted_qi(1) , shifted_qf(1) , qi(1), qf(1)]);
            sample_bounds.ymin = min([shifted_qi(2) , shifted_qf(2) , qi(2), qf(2)]);
            sample_bounds.ymax = max([shifted_qi(2) , shifted_qf(2) , qi(2), qf(2)]);
            sample_bounds.theta_min = min(qi(3),qf(3));
            sample_bounds.theta_max = max(qi(3),qf(3));           
            
        
    end

end

function q_new = Shift_Configuration(q,dx,dy)

    R = [cos(q(3)) , -sin(q(3));
         sin(q(3)) , cos(q(3))];
     
    q_new = q(1:2) + R*[dx;dy];

end
