clc;
close all;
clear;
%% Bruce: test simulation
% Initial q
q0 = [0.5;0.4;pi/4;0.0;0.3];
dqdt0 = [0.0;0.0;0.0;0.0;0.0];
x0 = [q0;dqdt0];
robot = Bruce(x0, @(x)terrain_map(x));
mtotal = robot.m_b + robot.m_fp + robot.m_hp + robot.m_lc;
g = robot.g;
kin_res = robot.foward_kinematics(q0, dqdt0);
Jac_res = robot.get_Jacobians(q0);
robot.get_dJtTimesdqdt(q0, dqdt0);
robot.guard_function(q0, dqdt0);
x = [q0;dqdt0];
X = [];
TT = [];
Tax = [];
Ctrl = [];
Contact = [];
Contact_Point = [];
Collision = [];
t = robot.time;
h_desired = 1.2;

% Simulation loop
i = 0;
while(t <= 7)
    % Check for collision
    if (robot.toe_contact == 1)
        disp('stance phase.');
        % Stance phase controller
        [tau_hip, f_leg, f_fp, f_hp] = stance_controller(x,mtotal,h_desired, g);

        % Step the robot
        x_next = robot.step_soft(x, [tau_hip; f_leg; f_fp; f_hp]);
    else
        disp('flight phase.');
        % Flight phase controller
        [tau_hip, f_leg, f_fp, f_hp] = flight_controller(x);

        % Step the robot
        x_next = robot.step_soft(x, [tau_hip; f_leg; f_fp; f_hp]);
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
    disp('total energy')
    disp(K)
    disp(V)
    disp(Total)
    X = [X, x_next];
    Tax = [Tax, t];
    TT = [TT, Total];
    Contact = [Contact, robot.toe_contact];
    Collision = [Collision, robot.toe_collision];
    Contact_Point = [Contact_Point, robot.contact_point];
    i = i + 1;
end
% %%
figure(1);
for i=1
    subplot(7,1,i);
    plot(Tax, X(i,:), 'Color','k', 'LineWidth', 2.0); 
    grid on;
end
% subplot(7,1,6);
% stairs(Tax, Contact, 'Color','b', 'LineWidth', 2.0);
% subplot(7,1,7);
% stairs(Tax, Collision, 'Color','r', 'LineWidth', 2.0);
% %%
% figure(45);
% plot(Tax , Contact_Point, 'LineWidth', 2.0);
% hold on;
% plot(Tax , X(1,:), 'Color','k', 'LineWidth', 2.0); 
% hold on;
% stairs(Tax , Collision, 'Color','r', 'LineWidth', 2.0);
% %%
% figure(3);
% plot(Tax, TT, 'LineWidth', 2.0);
% title("Energy");
% 
% saveAnimation(Tax, X, robot, 'test')