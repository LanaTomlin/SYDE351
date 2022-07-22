%% Global Parameters
mass = 20; %kg
mass_loaded = 40; %kg

%% Motor Parameters
La = 0.15*10^-3;
Ra = 0.035;
Kt = 8*10^-3;
Kb = 8*10^-3;
If = 1.15;
Cshaft = 10;



%% Suspension Parameters
c = 10;
k = 1000;
speed = 1; %m/s

wheel_separation = 0.5; %m
wheel_radius = 0.01; %m

input_torque = 9.8*mass_loaded*wheel_radius*0.6;
gear_ratio = input_torque/(17.5*10^-3);

%% Floor parameters
%note - these values are actually the distance travelled
Dtile = 0.3048;
Dgroove = 0.01;
Hgroove = 0.005;

%% Door parameters
%note - these values are actually the distance travelled
Ddoor = 0.05;
Hdoor = 0.03;
Ddelay = 15;

%% Robot Motion Parameters (Tune the robot motion here)
% %Tune the parameters found in the path generation functions at the end of
%this file
floorsig = sim("floorsignal.slx", 5);
sim("Suspension_System.slx", 5);


%% Path Planning
t = 0;
% Drive left for 15m, driving backwards
[v_l, v_r, t_new] = path_linear(13.5, false, 0, 0);
v_left = v_l;
v_right = v_r;
t = t + t_new;

% Rotate 90 deg CCW
[v_l, v_r, t_new] = path_rot(90.3999, false, 24*(0.46), 24*(0.46));
% Accelerate
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;
 
 % Move 10m Foreward
[v_l, v_r, t_new] = path_linear(10, true, 0, 0);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;
% 
% % Rotate 90 deg CW
[v_l, v_r, t_new] = path_rot(90, true, 0, 0);
% Accelerate
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;
% 
% % Move 8m Foreward
% [v_l, v_r, t_new] = path_linear(8, true);
% v_left = [v_left, v_l];
% v_right = [v_right, v_r];
% t = t + t_new;
% 
% % Generate total time and timeseries:
timescale = linspace(0, t, length(v_right));
% 
% Assuming starting pointing in poositive x direction
% t = linspace(0,100,1000);
% v_l = [linspace(0,20,400), linspace(0,0,40), linspace(3.52,3.52,50), linspace(0,0,30), linspace(20,20,480)];
% v_r = [linspace(0,20,400), linspace(0,0,40), linspace(-3.52,-3.52,50), linspace(0,0,30), linspace(20,20,480)];

left_motor_voltage = timeseries(v_left, timescale);
right_motor_voltage = timeseries(v_right, timescale);

path = sim("WheelDriveModel.slx", 60);

function [v_path_left, v_path_right, turn_time] = path_rot(target_angle, CW, right_voltage_start, left_voltage_start)
    accel_time = 5;
    turning_vel = 11.3; % deg/s
    accel_rot = accel_time*turning_vel/2;
    deccel_time = 10;
    deccel_rot = accel_time*turning_vel/2;
    turning_voltage = 5;

    if CW
        right_voltage = -turning_voltage + right_voltage_start;
        left_voltage = turning_voltage + left_voltage_start;
    else 
        right_voltage = turning_voltage + right_voltage_start;
        left_voltage = -turning_voltage + left_voltage_start;
    end

    % acel
    v_path_left = linspace(left_voltage_start, left_voltage, accel_time);
    v_path_right = linspace(right_voltage_start, right_voltage, accel_time);
    
    % Hold
    dist_full_speed = abs(target_angle) - accel_rot - deccel_rot;
    time_full_speed = dist_full_speed / turning_vel;
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, time_full_speed)];
    v_path_right = [v_path_right, linspace(right_voltage, right_voltage, time_full_speed)];
    
    % Decelerate
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, deccel_time/2)];
    v_path_right = [v_path_right, linspace(right_voltage, -right_voltage*0.46, deccel_time/2)];
   
    turn_time = accel_time + time_full_speed + deccel_time;
end

function [v_path_left, v_path_right, turn_time] = path_linear(target_distance, Forwards, right_voltage_start, left_voltage_start)
    drive_vel = 1.054;
    accel_time = 5;
    accel_dist = accel_time*drive_vel/2;
    deccel_time =  5;
    deccel_dist = deccel_time*drive_vel/2;
    drive_voltage = 24;

    if Forwards
        right_voltage = drive_voltage + right_voltage_start;
        left_voltage = drive_voltage + left_voltage_start;
    else 
        right_voltage = -drive_voltage + right_voltage_start;
        left_voltage = -drive_voltage + left_voltage_start;
    end
    % Accel
    v_path_left = linspace(left_voltage, left_voltage, accel_time);
    v_path_right = linspace(right_voltage, right_voltage, accel_time);
    
    % Hold
    dist_full_speed = abs(target_distance) - accel_dist - deccel_dist;
    time_full_speed = dist_full_speed / drive_vel;
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, time_full_speed)];
    v_path_right = [v_path_right, linspace(right_voltage, right_voltage, time_full_speed)];
    
    % Decelerate
    v_path_left = [v_path_left, linspace(left_voltage, 0, deccel_time)];
    v_path_right = [v_path_right, linspace(right_voltage, 0, deccel_time)];
    turn_time = accel_time + time_full_speed + deccel_time;


end