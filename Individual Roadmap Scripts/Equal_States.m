function equal = Equal_States(q1,q2)

    dq = round(norm(q2 - q1),4);
    
    if(dq == 0)
        equal = true;
    else
        equal = false;
    end

end
