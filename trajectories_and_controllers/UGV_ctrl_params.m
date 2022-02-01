%% UGV control parameters

% default params
ctrl.k1 = 10;
ctrl.k2 = 5;
ctrl.k3 = 10;

% our params
ctrl.ksi = 1.1;
ctrl.a = 60;
% considering linear/circular trajectory
ctrl.k1 = 2*ctrl.ksi*ctrl.a;
ctrl.k2 = (((ctrl.a^2)-(0.6^2))/0.8);
ctrl.k3 = ctrl.k1;

