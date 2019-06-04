function [x y xnl ynl xnr ynr theta odorSignall odorSignalr,conc_l,conc_r,capTime,success, randStart , baselineR , baselineL] = robot_test_A(x_start,y_start, theta_start, capRadius, varargin)
%ROBOT_TEST

% INPUTS
% x_start - Starting location of the bot, between 8 and 52
% y_start - Starting location of the bot, between -37 and 37
% theta_start - Starting heding of the bot, in radians
% capRadius - Distance from source considered a successful capture;


% OUTPUTS
% x - x trajectory of bot
% y - y trajectory
% xnl/r - L/R sensor x trajectory
% ynl/r - L/R sensor y trajectory
% theta - heading
% odorSignall/r - L/R sensor history
% capTime - Time to find source (NaN if unsuccessful
% success - logical var, 1 if captured, 0 if not
% randStart - Starting frame of the plume video
% baselineR/L - Odor baseline for each sensor

plotting = 0; % Turn realtime plotting on/off


%% Model Parameters
v_move = 4; % cm/s, speed while moving
l_d = 8; % cm, rough diameter of robot
l_s = 8; % cm, distance between sensors
l = hypot(l_s/2,l_d); % cm, distance from center of robot to sensor mount
gam = atan( (l_s/2)/l_d ); % inter-sensor angle, radians
theta_inc = deg2rad(30); % turn increment, radians
odorThres = 0.03; % difference between L/R sensor signals above which to initiate a turn
distThres = 10; % distance from center of robot to wall when wall is detected
noise_level = 0.0; % amplitude of normally distributed noise in sensor input
success = 0; % success variable - switched to 1 upon successful plume capture
randStart = 240*rand(); % random starting time to randomize PLIF plume 
central = 0; % port 1 if 0, port 2 if 1

%% Input argument pairs 

p = inputParser;
addOptional(p,'Port',central );
addOptional(p,'ls',l_s );

parse(p,varargin{:});

central = p.Results.Port;
l_s     = p.Results.ls;

%% Arena parameters

x_min = 0; % cm
x_max = 60; % cm

y_min = -45 ; % cm
y_max =  45; % cm

if central
    x_source = 10; % cm
    y_source = 0; % cm
    odorFun = @(x,y,t) odorFun_plume_center(x,y,t);
    
else
    x_source = 10; % cm
    y_source = 25; % cm
    odorFun = @(x,y,t) odorFun_plume_corner(x,y,t);
    
end


%% Time parameters

dt = 1/10; % seconds, simulation timestep
t_final = 75; % seconds

capTime = NaN; % Time to source, if successful

k_decay = log(2)/0.8; % sec, decay rate of sensor signal



%% Storage Vars and Initial Conditions

x = NaN(t_final/dt,1); % Body position, cm
y = NaN(t_final/dt,1);

xnl = NaN(t_final/dt,1); %Left nose position, cm
ynl = NaN(t_final/dt,1);
xnr = NaN(t_final/dt,1); % Right nose position, cm
ynr = NaN(t_final/dt,1);

theta = NaN(t_final/dt,1); % Heading, radians

odorSignall = NaN(t_final/dt,1); % Left signal, arb units
odorSignalr = NaN(t_final/dt,1); % Right signal, arb units

conc_l      = NaN(t_final/dt,1); % Left conc, arb units
conc_r      = NaN(t_final/dt,1); % Right conc, arb units

% ICs
theta(1) = theta_start; %
x(1) = x_start;
y(1) = y_start;
xnl(1) = x(1) + l*cos(theta(1) + gam);
xnr(1) = x(1) + l*cos(theta(1) - gam);
ynl(1) = y(1) + l*sin(theta(1) + gam);
ynr(1) = y(1) + l*sin(theta(1) - gam);

odorSignall(1) = 0;
odorSignalr(1) = 0;

conc_l(1) = odorFun(xnl(1) , ynl(1) , 1*dt+randStart );
conc_r(1) = odorFun(xnr(1) , ynr(1) , 1*dt+randStart );


%% Main Loop


%% Ten second warmup
for t = 2:10/dt
    
    % Maintain position and heading
    theta(t) = theta(t-1);
    x(t) = x(t-1);
    y(t) = y(t-1);
    xnl(t) = x(t) + l*cos(theta(1) + gam);
    xnr(t) = x(t) + l*cos(theta(1) - gam);
    ynl(t) = y(t) + l*sin(theta(1) + gam);
    ynr(t) = y(t) + l*sin(theta(1) - gam);
    
    % Let sensors equilibrate
    odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + noise_level*randn() );
    odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + noise_level*randn() );
    conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
    conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
    
end

%% Sample baseline
%Sample baseline right
baselineR = [];
for t = 10/dt+1 : 11/dt
    
    % Maintain position and heading
    theta(t) = theta(t-1);
    x(t) = x(t-1);
    y(t) = y(t-1);
    xnl(t) = x(t) + l*cos(theta(1) + gam);
    xnr(t) = x(t) + l*cos(theta(1) - gam);
    ynl(t) = y(t) + l*sin(theta(1) + gam);
    ynr(t) = y(t) + l*sin(theta(1) - gam);
    
    % Update sensor signals
    odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + noise_level*randn() );
    odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + noise_level*randn() );
    conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
    conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
    
    if mod(t*dt,0.2) == 0 & t*dt < 11
        baselineR = [baselineR ; odorSignalr(t)];
    end
end

baselineR = mean(baselineR);

%Sample baseline left
baselineL = [];
for t = 11/dt+1 : 12/dt
    
    % Maintain position and heading
    theta(t) = theta(t-1);
    x(t) = x(t-1);
    y(t) = y(t-1);
    xnl(t) = x(t) + l*cos(theta(1) + gam);
    xnr(t) = x(t) + l*cos(theta(1) - gam);
    ynl(t) = y(t) + l*sin(theta(1) + gam);
    ynr(t) = y(t) + l*sin(theta(1) - gam);
    
    % Update sensor signals
    odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + noise_level*randn() );
    odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + noise_level*randn() );
    conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
    conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
    
    if mod(t*dt,.2) == 0  & t*dt < 12
        baselineL = [baselineL ; odorSignall(t)];
    end
end
baselineL = mean(baselineL);
baseline = mean([baselineR;baselineL]);
baselineR = baseline;
baselineL = baseline;



while t < t_final/dt
    %% Stopping criterion
    if sqrt((x(t-1)-x_source).^2 + (y(t-1)-y_source).^2  )  < capRadius
        success = 1;
        capTime = t*dt;
        break
        %     elseif x(t-1) < 10
        %         success = 0;
        break
    end
    
    
    %% Wall avoidance
    %If robot near a wall, respond to IR sensors
    
    %Compute distances to walls
    dWall = [x(t-1) - x_min ...
        x_max  - x(t-1) ...
        y(t-1) - y_min ...
        y_max - y(t-1) ];
    
    
    if dWall(1) < distThres & abs(angdiff(pi, theta(t-1))) < pi/2
        wallAngle = angdiff(pi, theta(t-1));
        if wallAngle > -pi/2 & wallAngle < -pi/6    % If the wall is to the left, turn right
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        elseif wallAngle > pi/6 & wallAngle < pi/2    % If the wall is to the right, turn left
            t = t+1;
            % Update heading left
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnLeft(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        else                                        % Wall is ahead, turn right and back up
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            
            % Update position back
            for t = t+1:t+2
                % Update position back
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveBack(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        end
        
    elseif dWall(2) < distThres & abs(angdiff(0, theta(t-1))) < pi/2
        wallAngle = angdiff(0, theta(t-1));
        if wallAngle > -pi/2 & wallAngle < -pi/6    % If the wall is to the left, turn right
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        elseif wallAngle > pi/6 & wallAngle < pi/2    % If the wall is to the right, turn left
            t = t+1;
            % Update heading left
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnLeft(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        else                                        % Wall is ahead, turn right and back up
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position back
            for t = t+1:t+2
                % Update position back
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveBack(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        end
        
    elseif dWall(3) < distThres & abs(angdiff(3*pi/2, theta(t-1))) < pi/2
        wallAngle = angdiff(3*pi/2, theta(t-1));
        if wallAngle > -pi/2 & wallAngle < -pi/6    % If the wall is to the left, turn right
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        elseif wallAngle > pi/6 & wallAngle < pi/2    % If the wall is to the right, turn left
            t = t+1;
            % Update heading left
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnLeft(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        else                                        % Wall is ahead, turn right and back up
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position back
            for t = t+1:t+2
                % Update position back
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveBack(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        end
        
    elseif dWall(4) < distThres & abs(angdiff(pi/2, theta(t-1))) < pi/2
        wallAngle = angdiff(pi/2, theta(t-1));
        if wallAngle > -pi/2 & wallAngle < -pi/6    % If the wall is to the left, turn right
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        elseif wallAngle > pi/6 & wallAngle < pi/2    % If the wall is to the right, turn left
            t = t+1;
            % Update heading left
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnLeft(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        else                                        % Wall is ahead, turn right and back up
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position back
            for t = t+1:t+2
                % Update position back
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveBack(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        end
        
        %% Odor Navigation
        % Else if not near wall, sense odor and move accordingly
    else
        
        % Compute odor signals
        RealL = odorSignall(t-1) - baselineL;
        RealR = odorSignalr(t-1) - baselineR;
        
        
        if RealL - RealR > odorThres
            t = t+1;
            % Update heading left
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnLeft(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        elseif RealR - RealL > odorThres
            t = t+1;
            % Update heading right
            [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= turnRight(theta(t-1), x(t-1),y(t-1),l,gam,theta_inc);
            % Update odor signal
            odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
            odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
            conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
            conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            % Update position
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
        else % Go straight
            for t = t+1:t+2
                % Update position forward
                [theta(t), x(t),y(t),xnl(t),xnr(t),ynl(t),ynr(t)]= moveForward(theta(t-1), x(t-1),y(t-1),l,gam,v_move,dt);
                % Update odor signal
                odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
                odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
                conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
                conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
            end
            
        end
        
    end
    
    % Wait 300 ms
    
    for t = t+1:t+3
        theta(t) = theta(t-1);
        x(t) = x(t-1);
        y(t) = y(t-1);
        xnl(t) = x(t) + l*cos(theta(t-1) + gam);
        xnr(t) = x(t) + l*cos(theta(t-1) - gam);
        ynl(t) = y(t) + l*sin(theta(t-1) + gam);
        ynr(t) = y(t) + l*sin(theta(t-1) - gam);
        % Update odor signal
        odorSignall(t) = max( 0, odorSignall(t-1) + dt*odorFun(xnl(t) , ynl(t) , t*dt+randStart ) - k_decay*dt*odorSignall(t-1) + sqrt(dt)*noise_level*randn() );
        odorSignalr(t) = max( 0, odorSignalr(t-1) + dt*odorFun(xnr(t) , ynr(t) , t*dt+randStart ) - k_decay*dt*odorSignalr(t-1) + sqrt(dt)*noise_level*randn() );
        conc_l(t) = odorFun(xnl(t) , ynl(t) , t*dt+randStart );
        conc_r(t) = odorFun(xnr(t) , ynr(t) , t*dt+randStart );
    end
    
end

end



% Movement functions

function [theta, x,y,xnl,xnr,ynl,ynr]= turnRight(theta, x,y,l,gam,theta_inc)
theta = theta-theta_inc;
x = x;
y = y;
xnl = x + l*cos(theta + gam);
xnr = x + l*cos(theta - gam);
ynl = y + l*sin(theta + gam);
ynr = y + l*sin(theta - gam);
end

function [theta, x,y,xnl,xnr,ynl,ynr]= turnLeft(theta, x,y,l,gam,theta_inc)
theta = theta+theta_inc;
x = x;
y = y;
xnl = x + l*cos(theta + gam);
xnr = x + l*cos(theta - gam);
ynl = y + l*sin(theta + gam);
ynr = y + l*sin(theta - gam);
end

function [theta, x,y,xnl,xnr,ynl,ynr]= moveForward(theta, x,y,l,gam,v_move,dt)
theta = theta;
x = x + v_move*dt*cos(theta);
y = y + v_move*dt*sin(theta);
xnl = x + l*cos(theta + gam);
xnr = x + l*cos(theta - gam);
ynl = y + l*sin(theta + gam);
ynr = y + l*sin(theta - gam);
end


function [theta, x,y,xnl,xnr,ynl,ynr]= moveBack(theta, x,y,l,gam,v_move,dt)
theta = theta;
x = x - v_move*dt*cos(theta);
y = y - v_move*dt*sin(theta);
xnl = x + l*cos(theta + gam);
xnr = x + l*cos(theta - gam);
ynl = y + l*sin(theta + gam);
ynr = y + l*sin(theta - gam);
end
