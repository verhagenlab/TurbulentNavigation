%% Plume Specification 

% The following code initializes the dynamic plif plume
global plume
plume = h5read('11282017_10cms_bounded.h5','/dataset7');
plume = plume./max(max(max(plume)));
plume(plume<0) = 0;


%% Model and Port Specification 

portnum = 1;    % 0 if port 1, 1 if port 2
ls = 8;         % 8 cm or 16 cm intersensor distance

%% Angle and Replicate Specification

nruns = 50;     % Number of starting angles
nreps = 20;     % Number of replicates per starting angle (note: set to 1 if using static plume)

%% Main Loops

% Center start location
exp_B_center_turbulent = [];
startang = linspace(pi/2, 3*pi/2, nruns);

for i = 1:nruns
    for j = 1:nreps
        
        [x y xnl ynl xnr ynr theta odorSignall odorSignalr,conc_l,conc_r,capTime,success,randStart , baselineR , baselineL] = robot_test_B(48,0,startang(i),10,'Port',portnum, 'ls', ls);
        
        x = x(~isnan(x));
        y = y(~isnan(y));
        
        exp_B_center_turbulent(end+1 ).startangle = startang(i);
        exp_B_center_turbulent(end ).x = x(~isnan(x));
        exp_B_center_turbulent(end ).y = y(~isnan(x));
        exp_B_center_turbulent(end ).xnl = xnl(~isnan(x));
        exp_B_center_turbulent(end ).xnr = xnr(~isnan(x));
        exp_B_center_turbulent(end ).ynl = ynl(~isnan(x));
        exp_B_center_turbulent(end ).ynr = ynr(~isnan(x));
        exp_B_center_turbulent(end ).theta = theta(~isnan(x));
        exp_B_center_turbulent(end ).odor_l = odorSignall(~isnan(x));
        exp_B_center_turbulent(end ).odor_r = odorSignalr(~isnan(x));
        exp_B_center_turbulent(end ).conc_l = conc_l(~isnan(x));
        exp_B_center_turbulent(end ).conc_r = conc_r(~isnan(x));
        exp_B_center_turbulent(end ).baselinel = baselineL;
        exp_B_center_turbulent(end ).baseliner = baselineR;
        exp_B_center_turbulent(end ).capTime = capTime;
        exp_B_center_turbulent(end ).success = success;
        exp_B_center_turbulent(end ).tort = sum( hypot( diff(x),diff(y) ) )/pdist([x(1) y(1) ; x(end) y(end)]);
        exp_B_center_turbulent(end ).delta_signal_lr = ( odorSignall(~isnan(x)) - baselineL) - ( odorSignalr(~isnan(x)) - baselineR);

    end
end

% Corner start location
exp_B_corner_turbulent = [];
startang = linspace(pi, 3*pi/2, nruns);
for i = 1:nruns
    for j = 1:nreps

        [x y xnl ynl xnr ynr theta odorSignall odorSignalr,conc_l,conc_r,capTime,success,randStart , baselineR , baselineL] = robot_test_B(48,37,startang(i),10,'Port',portnum, 'ls', ls);
        
        x = x(~isnan(x));
        y = y(~isnan(y));
        
        exp_B_corner_turbulent(end+1 ).startangle = startang(i);
        exp_B_corner_turbulent(end).x = x(~isnan(x));
        exp_B_corner_turbulent(end).y = y(~isnan(x));
        exp_B_corner_turbulent(end).xnl = xnl(~isnan(x));
        exp_B_corner_turbulent(end).xnr = xnr(~isnan(x));
        exp_B_corner_turbulent(end).ynl = ynl(~isnan(x));
        exp_B_corner_turbulent(end).ynr = ynr(~isnan(x));
        exp_B_corner_turbulent(end).theta = theta(~isnan(x));
        exp_B_corner_turbulent(end).odor_l = odorSignall(~isnan(x));
        exp_B_corner_turbulent(end).odor_r = odorSignalr(~isnan(x));
        exp_B_corner_turbulent(end ).conc_l = conc_l(~isnan(x));
        exp_B_corner_turbulent(end ).conc_r = conc_r(~isnan(x));
        exp_B_corner_turbulent(end ).baselinel = baselineL;
        exp_B_corner_turbulent(end ).baseliner = baselineR;
        exp_B_corner_turbulent(end).capTime = capTime;
        exp_B_corner_turbulent(end).success = success;
        exp_B_corner_turbulent(end).tort = sum( hypot( diff(x),diff(y) ) )/pdist([x(1) y(1) ; x(end) y(end)]);
        exp_B_corner_turbulent(end ).delta_signal_lr = ( odorSignall(~isnan(x)) - baselineL) - ( odorSignalr(~isnan(x)) - baselineR);
    end
end




% Generate static plume
plume_avg = mean(plume,3);
for i = 1:3600
    plume(:,:,i) = plume_avg;
end




% Center start location, turbulent plume
exp_B_center_static = [];
startang = linspace(pi/2, 3*pi/2, nruns);

for i = 1:nruns
    for j = 1:nreps

        [x y xnl ynl xnr ynr theta odorSignall odorSignalr,conc_l,conc_r,capTime,success, randStart , baselineR , baselineL] = robot_test_B(48,0,startang(i),10,'Port',portnum, 'ls', ls);
        
        x = x(~isnan(x));
        y = y(~isnan(y));
        
        exp_B_center_static(end+1 ).startangle = startang(i);
        exp_B_center_static(end ).x = x(~isnan(x));
        exp_B_center_static(end ).y = y(~isnan(x));
        exp_B_center_static(end ).xnl = xnl(~isnan(x));
        exp_B_center_static(end ).xnr = xnr(~isnan(x));
        exp_B_center_static(end ).ynl = ynl(~isnan(x));
        exp_B_center_static(end ).ynr = ynr(~isnan(x));
        exp_B_center_static(end ).theta = theta(~isnan(x));
        exp_B_center_static(end ).odor_l = odorSignall(~isnan(x));
        exp_B_center_static(end ).odor_r = odorSignalr(~isnan(x));
        exp_B_center_static(end ).conc_l = conc_l(~isnan(x));
        exp_B_center_static(end ).conc_r = conc_r(~isnan(x));
        exp_B_center_static(end ).baselinel = baselineL;
        exp_B_center_static(end ).baseliner = baselineR;
        exp_B_center_static(end ).capTime = capTime;
        exp_B_center_static(end ).success = success;
        exp_B_center_static(end ).tort = sum( hypot( diff(x),diff(y) ) )/pdist([x(1) y(1) ; x(end) y(end)]);
        exp_B_center_static(end ).delta_signal_lr = ( odorSignall(~isnan(x)) - baselineL) - ( odorSignalr(~isnan(x)) - baselineR);
        
    end
end


% Corner start location, turbulent plume
exp_B_corner_static = [];
startang = linspace(pi, 3*pi/2, nruns);
for i = 1:nruns
    for j = 1:nreps

        [x y xnl ynl xnr ynr theta odorSignall odorSignalr,conc_l,conc_r,capTime,success, randStart , baselineR , baselineL] = robot_test_B(48,37,startang(i),10,'Port',portnum, 'ls', ls);
        
        x = x(~isnan(x));
        y = y(~isnan(y));
        
        exp_B_corner_static(end+1 ).startangle = startang(i);
        exp_B_corner_static(end).x = x(~isnan(x));
        exp_B_corner_static(end).y = y(~isnan(x));
        exp_B_corner_static(end).xnl = xnl(~isnan(x));
        exp_B_corner_static(end).xnr = xnr(~isnan(x));
        exp_B_corner_static(end).ynl = ynl(~isnan(x));
        exp_B_corner_static(end).ynr = ynr(~isnan(x));
        exp_B_corner_static(end).theta = theta(~isnan(x));
        exp_B_corner_static(end).odor_l = odorSignall(~isnan(x));
        exp_B_corner_static(end).odor_r = odorSignalr(~isnan(x));
        exp_B_corner_static(end ).conc_l = conc_l(~isnan(x));
        exp_B_corner_static(end ).conc_r = conc_r(~isnan(x));
        exp_B_corner_static(end ).baselinel = baselineL;
        exp_B_corner_static(end ).baseliner = baselineR;
        exp_B_corner_static(end).capTime = capTime;
        exp_B_corner_static(end).success = success;
        exp_B_corner_static(end).tort = sum( hypot( diff(x),diff(y) ) )/pdist([x(1) y(1) ; x(end) y(end)]);
        exp_B_corner_static(end ).delta_signal_lr = ( odorSignall(~isnan(x)) - baselineL) - ( odorSignalr(~isnan(x)) - baselineR);
        
    end
end







