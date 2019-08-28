%% Initialization

addpath('C:\Users\Cmang\Desktop\Research\Central Planner','C:\Users\Cmang\Desktop\Research\Individual Roadmap Scripts',...
    'C:\Users\Cmang\Desktop\Research\Local Controller');
load("PRM_V6.mat","G","H","valid_indices","Qi","Qf");

N = size(Qi,1);
vehicle_dimensions = struct('length',3.2,'width',0.8);

%% 1. Planning
[~ , wp] = dRRT_star(Qi,Qf,G,H,valid_indices);

%% 2. Control

Wr_t = cell(N,1); % Angular Velocity Reference Signals
Xr_t = cell(N,1); % Pose Reference Signals
dt = 0.01; % Sampling Time
Vr_t = cell(N,1); 
v = zeros(1,N); % Initial speeds
v_optimal = 5;
for i = 1:length(wp)
    [Vr_t{i}, Wr_t{i} , Xr_t{i}] = Target_Velocities(wp{i} , v_optimal , dt);
    v(i) = Vr_t{i}(2,1);
end

%% 3. Visualization
sc = Intersection_Scenario_1;
vehicle_actors = Spawn_Vehicles(Qi,sc,vehicle_dimensions);
plot(sc);
X_t = [ Qi(:,1:3)'];


for i = 1:length(Xr_t{1})
    
    for j = 1:N
        u = Local_Controller(X_t(:,i),t,dt,Xr_t{j},V_t{i},W_t{i});
        X_t(:,j) = Xr_t{j}(:,i);
    end
    sc = Update_Poses(sc,X_t);
    updatePlots(sc);
    pause(0.00001);
end

%% Helper Functions
% Returns actor objects
function vehicles = Spawn_Vehicles(Qi,sc,vehicle_dimensions)

    for i = 1:size(Qi,1)
        vehicles(i) = actor(sc,'Position',[ Qi(i,1:2) , 0],'Yaw',rad2deg(Qi(i,3)),...
                     'Length',vehicle_dimensions.length,'Width',vehicle_dimensions.width);
                          
        
    end

end

% Updates actor positions
function sc = Update_Poses(sc,X_t)
    for i = 1:length(sc.Actors)
        if not( any(isnan(X_t(1:2,i))))
            sc.Actors(1,i).Position = [ X_t(1:2,i)' , 0];
            sc.Actors(1,i).Yaw = rad2deg(X_t(3,i));
        end
    end
end