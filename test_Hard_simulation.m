clc;
close all;
clear;
%% Bruce: test simulation
% Initial q
q0 = [0.5;0.5;pi/7;pi/7;0.3];    
dqdt0 = [0.01;0.0;0.0;0.0;0.0];
x0 = [q0;dqdt0];
robot = Bruce(x0, @(x)terrain_map(x));
mtotal = robot.m_b + robot.m_fp + robot.m_hp + robot.m_lc;
g = robot.g;
kin_res = robot.foward_kinematics(q0, dqdt0);
Jac_res = robot.get_Jacobians(q0);
robot.get_dJtTimesdqdt(q0, dqdt0);
robot.guard_function(q0, dqdt0);
x = [q0;dqdt0];
h_desired = 0.45; % desire height
v_desired = 0.5; % desire velocity
X = []; 
TT = [];
Tax = [];
Ctrl = [];
Contact = [];
Contact_Point = [];
Collision = [];
f_leg_data = [];
f_fp_data = [];
f_hp_data = [];
t = robot.time;
T_ss = NaN;  % stance phase start time
T_s_array = [];  % Array to store stance phase durations
step_length_array = [];  % Array to store step lengths
step_time_array = [];  % Array to store the time points when step lengths are recorded
com_position_prev = x(1);  % Assume the first element of x is the horizontal position
thrust_active = false; % 初始化为 false
% Simulation loop
i = 0;
while(t <= 10)
    
    % Check for collision
    if (robot.toe_contact == 1)
        disp('stance phase.');
        % recording time
        if isnan(T_ss)
            T_ss = t;  % stance phase start time
        end
        % Stance phase controller
        [tau_hip, f_leg, f_fp, f_hp] = stance_controller(x,mtotal,g,h_desired,v_desired);
        % Step the robot
        x_next = robot.step_hard(x, [tau_hip; f_leg; f_fp; f_hp]);
   
    else
        disp('flight phase.');
        % recording time
        if ~isnan(T_ss)
            T_s = t - T_ss;  % stance phase duration time
            T_s_array = [T_s_array, T_s];  % Store the duration
            T_ss = NaN;  % reset stance phase start time
        else
            T_s = 0;  % Default value if T_s is not yet defined
        end 
        % Flight phase controller
        [tau_hip, f_leg, f_fp, f_hp] = flight_controller(x,T_s,v_desired,thrust_active);
        % Step the robot
        x_next = robot.step_hard(x, [tau_hip; f_leg; f_fp; f_hp]);
    end
        
    % Update state and time
    x = x_next;
    if mod(i, 100) == 0
        kin_res = robot.foward_kinematics(x(1:5), x(6:10));
        robot.visualize(kin_res);
    end
    t = robot.time;
    
    % Log data
    [K, V, Total] = robot.get_energy(x(1:5), x(6:10));
    X = [X, x_next];
    Tax = [Tax, t];
    TT = [TT, Total];
    Contact = [Contact, robot.toe_contact];
    Collision = [Collision, robot.toe_collision];
    Contact_Point = [Contact_Point, robot.contact_point];
    f_leg_data = [f_leg_data, f_leg];
    f_fp_data = [f_fp_data, f_fp];
    f_hp_data = [f_hp_data, f_hp];
    
    % Calculate and record step length
    if ~isnan(T_ss) && robot.toe_contact == 1
        com_position_curr = x(1);  % Assume the first element of x is the horizontal position
        step_length = abs(com_position_curr - com_position_prev);  % Horizontal distance
        step_length_array = [step_length_array, step_length];
        step_time_array = [step_time_array, t];  % Record the time point
        com_position_prev = com_position_curr;  % Update previous position
    end
    
    i = i + 1;
end

%% figure
figure(1);
plot(Tax, X(2,:), 'b', 'LineWidth', 2); 
hold on;

% h_max 
h_max_value = 0.4462; 
h_max_vector = repmat(h_max_value, size(Tax));
plot(Tax, h_max_vector, 'r--', 'LineWidth', 2);

% h_desired 
h_desired_value = 0.506; 
h_desired_vector = repmat(h_desired_value, size(Tax));
plot(Tax, h_desired_vector, 'Color', [0 0.3 0],'LineStyle', '--', 'LineWidth', 2);  % 使用RGB值 [0.5, 0.2, 0.8] 作为颜色

% 设置图表属性
xlim([0 10]);
ylim([0.25, 0.65]);
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 40);
ylabel('Hopping height (m)', 'FontName', 'Times New Roman', 'FontSize', 40);
% legend('$h$', '$h_\mathrm{desired}$', 'FontSize', 40, 'Interpreter', 'latex');
legend('$h$', '$h_\mathrm{max}$', '$h_\mathrm{desired}$','FontSize', 40, 'Interpreter', 'latex');
grid on;
ax = gca;
set(ax, 'FontSize', 20);

figure(2);
plot(Tax, f_leg_data, 'b', 'LineWidth', 2); 
hold on;
plot(Tax, f_fp_data, 'r', 'LineWidth', 3); 
hold on;
plot(Tax, f_hp_data, 'k', 'LineWidth', 2);
grid on;
xlim([0 10]);
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 40);
ylabel('F (N)', 'FontName','Times New Roman', 'FontSize', 40);
legend('$f_\mathrm{leg}$', '$f_\mathrm{fp}$','$f_\mathrm{hp}$', 'FontSize',40,'Interpreter','latex');
set(gca, 'FontSize', 20);

% inset plot
axes('Position', [.3, .4, .25, .25]); % position and size
plot(Tax, f_fp_data, 'r', 'LineWidth', 3); 
hold on;
plot(Tax, f_hp_data, 'k', 'LineWidth', 2);
xlim([0 2]); 
ylim([0 15]); 
grid on;
% xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 20);
% ylabel('F (N)', 'FontName','Times New Roman', 'FontSize', 20); 
% legend('$f_\mathrm{fp}$','$f_\mathrm{hp}$', 'FontSize',20,'Interpreter','latex');
set(gca, 'FontSize', 20);

figure(3);
plot(Tax, TT, 'b', 'LineWidth', 2); 
hold on;
xlim([0 10]);
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 40);
ylabel('Energy (J)', 'FontName','Times New Roman', 'FontSize', 40);

figure(4);
plot(Tax, X(3,:), 'b', 'LineWidth', 2); 
hold on;

% h_desired 
h_desired_value = 0.08; 
h_desired_vector = repmat(h_desired_value, size(Tax));
plot(Tax, h_desired_vector, 'Color', [0 0.3 0],'LineStyle', '--', 'LineWidth', 2); 

% 设置图表属性
xlim([0 10]);
ylim([0.25, 0.5]);
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 40);
ylabel('Body orientation (rad)', 'FontName','Times New Roman', 'FontSize', 40);
legend('$\theta_{\mathrm{b}}$', '$\theta_{\mathrm{b}_\mathrm{desired}}$','FontSize', 40, 'Interpreter', 'latex');
grid on;
ax = gca;
set(ax, 'FontSize', 20);

% 添加网格
grid on;
ax = gca;
set(ax, 'FontSize', 20);  % 设置坐标轴字体大小
set(ax, 'GridLineStyle', '--');  % 设置网格线为虚线
set(ax, 'GridColor', [0.8 0.8 0.8]);  % 设置网格线颜色为灰色

figure(5);
y_labels = {'Position X (m)', 'Position Y (m)', 'Orentation body (rad)', 'Orentation leg (rad)', 'Leg length (m)'};

for i = 1:5
    subplot(5, 1, i);
    plot(Tax, X(i, :), 'Color', 'k', 'LineWidth', 2.0);
    grid on;
    xlabel('Time (s)');
    ylabel(y_labels{i});  
end

figure(6);
y_labels = {'Velocity X (m/s)', 'Velocity Y (m/s)', 'Body angular velocity (rad/s)', 'Leg angular velocity (rad/s)', 'Leg length velocity (m/s)'};

for i = 6:10
    subplot(5, 1, i-5); % 修正索引
    plot(Tax, X(i, :), 'Color', 'k', 'LineWidth', 2.0);
    grid on;
    xlabel('Time (s)');
    ylabel(y_labels{i-5}); % 修正索引
end
%
% figure(2);
% plot(Tax, f_leg_data, 'LineWidth', 2.0);
% % title('f_leg vs. Time');
% xlabel('Time (s)');
% ylabel('Leg force (N)');
% grid on;
% 
% figure(3);
% plot(Tax, f_fp_data, 'LineWidth', 2.0);
% % title('f_fp vs. Time');
% xlabel('Time (s)');
% ylabel('Front propeller force (N)');
% grid on;
% 
% figure(4);
% plot(Tax, f_hp_data, 'LineWidth', 2.0);
% % title('f_hp vs. Time');
% xlabel('Time (s)');
% ylabel('Hind propeller force (N)');
% grid on;

% saveAnimation(Tax, X, robot, 'test')
% subplot(7,1,6);
% stairs(Tax, Contact, 'Color','b', 'LineWidth', 2.0);
% subplot(7,1,7);
% stairs(Tax, Collision, 'Color','r', 'LineWidth', 2.0);