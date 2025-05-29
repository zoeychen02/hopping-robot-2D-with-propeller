function [tau_hip, f_leg, f_fp, f_hp, thrust_active, thrust_duration] = flight_controller(x, T_s, v_desired, thrust_active, thrust_duration)
    
    %% define state
    robot_pos = [x(1); x(2)];
    robot_orient = [x(3); x(4)];
    robot_vel = [x(6); x(7)];
    robot_angvel = [x(8); x(9)];

    %% Physical limits
    f_min = 0.1; % Minimum thrust
    f_max = 30;  % Maximum thrust
    kp = 100; % hip torque P gain
    kd = 1; % hip torque D gain
    kr = 0.4; % stepper controller gain
    propKp = 50; % propeller P gain
    propKd = 5; % propeller D gain 
    Kp_leg = 60; % leg P gain
    Kd_leg = 1; % leg D gain
    l_p = 0.3;  % length of propeller arm
    l_leg_max = 0.3;% max leg length

    %% Leg control
    f_leg = Kp_leg * (0.3 - x(5)) - Kd_leg * x(10);

    %% Raibert stepping controller
    if abs(robot_vel(1) - v_desired) > 0.4
        error = sign((robot_vel(1) - v_desired)) * 0.4;
    else
        error = robot_vel(1) - v_desired;
    end
    beta_desired = asin((robot_vel(1) * T_s / 2) / l_leg_max) + kr * (error);
    beta_current = robot_orient(2) + robot_orient(1);
    tau_hip = kp * (-beta_current - beta_desired) + kd * (0.0 - robot_angvel(2));
    
    %% propeller control
    if abs(robot_orient(1)) > 0.1 
        tau_p = (0.08 - robot_orient(1)) * propKp + (0 - robot_angvel(1)) * propKd;
        tau_p_line = max(-1.8, min(1.8, tau_p));      
        % Parameters for linprog
        c = [1; 1];  % Objective function coefficient (minimum f1 + f2)
        Aeq = [-l_p, l_p];  
        beq = [tau_p_line]; 
        lb = [f_min; f_min]; 
        ub = [f_max; f_max];  

        A = []; 
        b = [];  

        % linprog
        options = optimoptions('linprog', 'Algorithm', 'dual-simplex');  
        [f, fval, exitflag, output] = linprog(c, A, b, Aeq, beq, lb, ub, options);

        % Output
        if exitflag == 1
            f_fp = f(1);
            f_hp = f(2); 
           else
        f_fp = 0.1;
        f_hp = 0.1; 
        end
    
    else
    f_fp = 0;
    f_hp = 0;
%     if robot_vel(2) > 0
%         
%         f_fp = 0.4;
%         f_hp = 0.4;
%     else
%         
%         f_fp = 0;
%         f_hp = 0; 
%     end
end
     
%     %% Additional upward thrust logic
%     additional_thrust = 5; % Adjust this value as needed
%     max_thrust_duration = 0.1; % 0.1 seconds
% 
%     % Check if orientation is within the desired range
%     if abs(robot_orient(1)) < 0.1 && ~thrust_active
%         % Start applying additional thrust
%         thrust_active = true;
%         thrust_duration = 0; % Reset the duration counter
%     end
% 
%     % If thrust is active, apply additional thrust and update duration
%     if thrust_active
%         f_fp = f_fp + additional_thrust;
%         f_hp = f_hp + additional_thrust;
%         
%         % Update thrust duration
%         thrust_duration = thrust_duration + T_s;
%         
%         % Check if thrust duration has exceeded the limit
%         if thrust_duration >= max_thrust_duration
%             thrust_active = false; % Stop applying thrust
%             thrust_duration = 0; % Reset the duration counter
%         end
%     end

    %% Display results
%       disp(f_fp);
%       disp(f_hp);
%       disp(f_leg);
%       disp(x(6));