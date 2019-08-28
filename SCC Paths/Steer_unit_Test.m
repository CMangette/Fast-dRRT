qi = [0;0;0;0];
qf = [10;4;pi/6;0];

TP = Turning_Param(0.5,1);

path = Steer(qi,qf,TP);
s = 0:0.01:path.path_param.length;
q_crit = SCC_State(path,s);
figure;plot(q_crit(1,:),q_crit(2,:));