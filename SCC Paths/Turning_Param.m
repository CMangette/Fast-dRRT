function param = Turning_Param(k_max,sigma_max)
%TURNING_PARAM returns a structure containing turning parameters and
%constraints used for computing a SCC path

    param.k_max = k_max;
    param.sigma_max = sigma_max;
    param.delta_min = (k_max^2)/sigma_max;
    param.inner_rad = 1 / k_max;
    
    % intermediate config qi
    scale = sqrt(pi / sigma_max);
    
    xi = scale*fresnelc(sqrt(param.delta_min / pi ));
    yi = scale*fresnels(sqrt(param.delta_min / pi ));
    theta_i = param.delta_min / 2;
    k_i = k_max;
    
    param.qi = [xi ; yi ; theta_i ; k_i];
    
    % Turning Circle Center
    cx = xi - sin(theta_i) / k_max;
    cy = yi + cos(theta_i) / k_max;
    
    param.om_left = [cx ; cy];
    param.om_right = [cx ; -cy];
    
    param.outer_rad = norm([cx , cy]);
    param.gamma = atan2(cx , cy);
    
end

