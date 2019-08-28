function [G,valid_indices] = Remove_Infeasible_States(G,H,q_goal)

    goal_idx = Query_State(G,q_goal);

    valid_indices = [];
    for i = 1:length(G)
        
        if isinf(H(i,goal_idx))
            
            G = DeRef(G,i);
            
        else
            valid_indices = [valid_indices , i];
        end
    end

end

% Removes all reference in G to and from vertex i
function G = DeRef(G,idx)

% Remove all pointers to idx from other nodes
    parents = G(idx).prev;

    for i = 1:length(parents)
    
        childIdx = find(G(parents(i)).next == idx);
        G(parents(i)).ctg(childIdx) = [];
        G(parents(i)).next(childIdx) = [];
        G(parents(i)).path(childIdx) = [];
        G(parents(i)).inflated_path(childIdx) = [];
    
    end
    
    children = G(idx).next;
    
    for j = 1:length(children)
        
        parentIdx = G(children(j)).prev == idx;
        G(children(j)).prev(parentIdx) = [];
        
    end
    
    G(idx) = struct('state',[],'prev',[],'next',[],'ctg',[],'path',[],'inflated_path',[]);

end

