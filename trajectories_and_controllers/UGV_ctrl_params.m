%% UGV control parameters
ctrl.ksi = 1.1;
ctrl.a = 60;
%considering linear/circular trajectory

ctrl.k1 = 2*ctrl.ksi*ctrl.a;
ctrl.k2 = (((ctrl.a^2)-(constant_desired_velocity_w^2))/constant_desired_velocity_v);
ctrl.k3 = ctrl.k1;