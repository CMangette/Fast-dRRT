function x_s = Path_Reference(wp)

    % PATH_REFERENCE 
    % Returns the SCC Path segment described by the set of input waypoints
    S = 0;
    
    TP = Turning_Param(0.5,2);
    
    x_s = [];
    for i = 1:length(wp) - 1
       
        path_local = Steer(wp(i).state,wp(i+1).state,TP);
        
        
        if path_local.type == STRAIGHT
           
            x = linspace(path_local.qi(1),path_local.qf(1),50);
            y = linspace(path_local.qi(2),path_local.qf(2),50);
            theta = ones(1,50)*path_local.qi(3);
            kappa = zeros(1,50);
            s = S + linspace(0,path_local.path_param.length,50);
            
            path_states = [x ; y ; theta ; kappa ; s];
        else
            s = linspace(0,path_local.path_param.length,50);
        
            path_states = SCC_Path(path_local,s);
            s = s + S;
            path_states = [path_states ; s];
        
        end
        x_s = [x_s , path_states];
        S = x_s(end);
    end

end


function s = STRAIGHT()
    s = 5;
end