%% UGV control parameters
ctrl.ksi = 0.01;
ctrl.a = 5;
%considering linear/circular trajectory

ctrl.k1 = 2*ctrl.ksi*ctrl.a;
ctrl.k2 = (((ctrl.a^2)-(constant_desired_velocity_w^2))/constant_desired_velocity_v);
ctrl.k3 = ctrl.k1;