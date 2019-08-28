%% Setup
addpath('C:\Users\Cmang\Desktop\Research\SCC Paths');
load('StateLookup.mat');

% Roadmap Graph
 G = cell(12,1);
 H = cell(12,1);
 valid_indices = cell(12,1);

sequence = [1,1,6,1;
            1,2,4,2;
            1,3,2,3;
            3,1,8,1;
            3,2,6,2;
            3,3,4,3;
            5,1,2,1;
            5,2,8,2;
            5,3,6,3;
            7,1,4,1;
            7,2,2,2;
            7,3,8,3];
            
intention = [3;2;1;3;2;1;3;2;1;3;2;1];

IP = polyshape([-12 -12 12 12],[-12 12 12 -12]);

vehicle_dimensions = [-1.8 , -1.8 , 1.8 , 1.8;
                      0.9 -0.9 -0.9 0.9];

N = 225; % Number of points in roadmap
r = 12; % maxumim connection Radius

Qi = zeros(12,4);
Qf = zeros(12,4);
%% Pre-computation Stage

for i = 1:size(sequence,1)
    
    qi = StateLookUp{sequence(i,1),sequence(i,2)}(1:3);
    qf = StateLookUp{sequence(i,3),sequence(i,4)}(1:3);
    
    Qi(i,1:3) = qi;
    Qf(i,1:3) = qf;
%     % Roadmap Construction Step
    G{i} = CC_PRM(qi , qf , N , r , intention(i) , vehicle_dimensions);
%     % Precomputation of travel costs
    H{i} = Travel_Costs(G{i});
    [ G{i} , valid_indices{i} ] = Remove_Infeasible_States(G{i},H{i},[qf,0]); 
    
end
