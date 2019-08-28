function p_r = Target_Poses(s_t,path)

    p_r = zeros(4,size(s_t,2));
    for i = 1:length(s_t(1,:))
        
        p_r(1,i) = s_t(1,i);
        p_r(2,i) = interp1(path(5,:),path(1,:),s_t(2,i));
        p_r(3,i) = interp1(path(5,:),path(2,:),s_t(2,i));
        p_r(4,i) = interp1(path(5,:),path(3,:),s_t(2,i));
        
    end
end

