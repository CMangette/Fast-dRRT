function idx = Query_State(G,q)
    idx = 0;
    for i = 1:length(G)
        if(Equal_States(G(i).state,q))
            idx = i;
        end
    end
end

