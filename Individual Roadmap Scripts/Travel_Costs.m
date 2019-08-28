function H = Travel_Costs(G)
%TRAVEL_COSTS
    H = zeros(length(G));

    for i = 1:length(G)
        for j = 1:length(G)
            
            if(i == j)
                
                continue;
                
            else
                
                H(i,j) = BreadthFirstSearch(G,i,j);
                
            end
            
        end
    end
end

