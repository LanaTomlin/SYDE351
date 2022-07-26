%% Global Parameters
mass = 20; %kg
mass_loaded = 20; %kg

%% Motor Parameters
La = 0.15*10^-3;
Ra = 0.035;
Kt = 8*10^-3;
Kb = 8*10^-3;
If = 1.15;
Cshaft = 1;

%% Suspension Parameters
c = 10;
k = 10;
speed = 0.508; %m/s

wheel_separation = 0.5; %m
wheel_radius = 0.01; %m

input_torque = 0; % No additional load other than mechanical resistance.
gear_ratio = 3;

%% Floor parameters
%note - these values are actually the distance travelled
Dtile = 0.3048;
Dgroove = 0.01;
Hgroove = 0.005;

%% Door parameters
%note - these values are actually the distance travelled
Ddoor = 0.05;
Hdoor = 0.03;
Ddelay = 0;

%% Robot Motion Parameters (Tune the robot motion here)
% %Tune the parameters found in the path generation functions at the end of
%this file
floorsig = sim("floorsignal.slx", 5/speed);
doorsig = sim("DoorSignal.slx", 5/speed + 0.2);

sim("Suspension_System.slx", 5/speed + 0.3);




%% Path Planning
t = 0;
% Drive left for 15m, driving backwards
[v_l, v_r, t_new] = path_linear(15, false);
v_left = v_l;
v_right = v_r;
t = t + t_new;

% %Stop
[v_l, v_r, t_new] = path_stop(0, 5);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
 t = t + t_new;

% Rotate 90 deg CCW
[v_l, v_r, t_new] = path_rot(90, false);
% Accelerate
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% %Stop
[v_l, v_r, t_new] = path_stop(0, 5);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
 t = t + t_new;
 
%  % Move 10m Foreward
[v_l, v_r, t_new] = path_linear(10, true);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% %Stop
[v_l, v_r, t_new] = path_stop(0, 5);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
 t = t + t_new;

% % Rotate 90 deg CW
[v_l, v_r, t_new] = path_rot(90, true);
% Accelerate
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% %Stop
[v_l, v_r, t_new] = path_stop(0, 5);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
 t = t + t_new;

% % Move 8m Foreward
[v_l, v_r, t_new] = path_linear(8, true);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% %Stop
[v_l, v_r, t_new] = path_stop(0, 5);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
 t = t + t_new;

% Generate total time and timeseries:
timescale = linspace(0, t, length(v_right));

% Assuming starting pointing in poositive x direction
% t = linspace(0,100,1000);
% v_l = [linspace(0,20,400), linspace(0,0,40), linspace(3.52,3.52,50), linspace(0,0,30), linspace(20,20,480)];
% v_r = [linspace(0,20,400), linspace(0,0,40), linspace(-3.52,-3.52,50), linspace(0,0,30), linspace(20,20,480)];

left_motor_voltage = timeseries(v_left, timescale);
right_motor_voltage = timeseries(v_right, timescale);

path = sim("WheelDriveModel.slx", 1000);

function [v_path_left, v_path_right, turn_time] = path_rot(target_angle, CW)
    accel_time = 3;
    turning_vel = 63.17; % deg/s
    accel_rot = 21;
    deccel_time = 3;
    deccel_rot = 21;
    turning_voltage = 5;
    time_scaler = 100; % 1/s

    if CW
        right_voltage = -turning_voltage;
        left_voltage = turning_voltage;
    else 
        right_voltage = turning_voltage;
        left_voltage = -turning_voltage;
    end

    % acel
    v_path_left = linspace(0, left_voltage, accel_time*time_scaler);
    v_path_right = linspace(0, right_voltage, accel_time*time_scaler);
    
    % Hold
    dist_full_speed = abs(target_angle) - accel_rot - deccel_rot;
    time_full_speed = dist_full_speed / turning_vel;
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, time_full_speed*time_scaler)];
    v_path_right = [v_path_right, linspace(right_voltage, right_voltage, time_full_speed*time_scaler)];
    
    % Decelerate
    v_path_left = [v_path_left, linspace(left_voltage, 0, deccel_time*time_scaler)];
    v_path_right = [v_path_right, linspace(right_voltage, 0, deccel_time*time_scaler)];
   
    turn_time = accel_time + time_full_speed + deccel_time;
end

function [v_path_left, v_path_right, path_time] = path_linear(target_distance, Forwards)
    drive_vel = 0.508;
    accel_time = 7;
    accel_dist = 2.1;
    deccel_time = 7;
    deccel_dist = 1.28;
    drive_voltage = 24;
    time_scaler = 100; % 1/s

    if Forwards
        right_voltage = drive_voltage;
        left_voltage = drive_voltage;
    else 
        right_voltage = -drive_voltage;
        left_voltage = -drive_voltage;
    end
    % Accel
    v_path_left = linspace(0, left_voltage, accel_time*time_scaler);
    v_path_right = linspace(0, right_voltage, accel_time*time_scaler);
    
    % Hold
    dist_full_speed = abs(target_distance) - accel_dist - deccel_dist;
    time_full_speed = dist_full_speed / drive_vel;
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, time_full_speed*time_scaler)];
    v_path_right = [v_path_right, linspace(right_voltage, right_voltage, time_full_speed*time_scaler)];
    
    % Decelerate
    v_path_left = [v_path_left, linspace(left_voltage, 0, deccel_time*time_scaler)];
    v_path_right = [v_path_right, linspace(right_voltage, 0, deccel_time*time_scaler)];
    path_time = accel_time + time_full_speed + deccel_time;

end

function [v_path_left, v_path_right, turn_time] = path_stop(volt_stop, time_stop)
    time_scaler = 100; % 1/s

    v_path_left = linspace(volt_stop, volt_stop, time_stop*time_scaler);
    v_path_right = linspace(volt_stop, volt_stop, time_stop*time_scaler);
    turn_time = time_stop;
end
