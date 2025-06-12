clc;
close all;
clear;
Tf = 5.;
%% Bruce: soft simulation
% Initial q
q0 = [0.5;0.8;0.0;0.0;0.2];
dqdt0 = [0.0;0.0;0.0;0.0;0.0];
x0 = [q0;dqdt0];
robot_soft = Bruce(x0, @(x)terrain_map(x));

x = [q0;dqdt0];
X_s = [];       
TT_s = [];      
Tax_s = [];     
Ctrl_s = [];    
Contact_s = []; 
Contact_Point_s = []; 
Collision_s = [];     
t_s = robot_soft.time;
while(t_s <= Tf)
    tau_hip = -50 * (x(4)-0.0) - 1.0*x(9);
    if(robot_soft.toe_contact == 1)
        % stance phase
        if(x(7) <= 0.)
            f_leg = 0 - 1000 * (x(5)-0.3) - 10.0*x(10);
        else
            f_leg = 10 - 1000 * (x(5)-0.3) - 10.0*x(10);
        end
    else
        f_leg = -500 * (x(5)-0.2) - 10.0*x(10);
    end
    body_ctrl = -800 * (x(3) - 0.) - 70 * x(8);
    f_fp = 0.;
    f_hp = 0.;
    
    x_next = robot_soft.step_soft(x, [tau_hip; f_leg; f_fp; f_hp]);
    [K, V, Total] = robot_soft.get_energy(x(1:5), x(6:10));
    X_s = [X_s, x_next];
    Tax_s = [Tax_s, t_s];
    TT_s = [TT_s, Total];
    Contact_s = [Contact_s robot_soft.toe_contact];
    Collision_s = [Collision_s robot_soft.toe_collision];
    Contact_Point_s = [Contact_Point_s robot_soft.contact_point];
    x = x_next;
    t_s = robot_soft.time;
end
%% Bruce: hard simulation
% Initial q
q0 = [0.5;0.8;0.0;0.0;0.2];
dqdt0 = [0.0;0.0;0.0;0.0;0.0];
x0 = [q0;dqdt0];
robot_hard = Bruce(x0, @(x)terrain_map(x));

x = [q0;dqdt0];
X_h = [];
TT_h = [];
Tax_h = [];
Ctrl_h = [];
Contact_h = [];
Contact_Point_h = [];
Collision_h = [];
t_h = robot_hard.time;
while(t_h <= Tf)
    tau_hip = -50 * (x(4)-0.0) - 1.0*x(9);
    if(robot_hard.toe_contact == 1)
        % stance phase
        if(x(7) <= 0.)
            f_leg = 0 - 1000 * (x(5)-0.3) - 10.0*x(10);
        else
            f_leg = 10 - 1000 * (x(5)-0.3) - 10.0*x(10);
        end
    else
        f_leg = -500 * (x(5)-0.2) - 10.0*x(10);
    end
    body_ctrl = -800 * (x(3) - 0.) - 70 * x(8);
    f_fp = 0.;
    f_hp = 0.;
    
    x_next = robot_hard.step_hard(x, [tau_hip; f_leg; f_fp; f_hp]);
    [K, V, Total] = robot_hard.get_energy(x(1:5), x(6:10));
    X_h = [X_h, x_next];
    Tax_h = [Tax_h, t_h];
    TT_h = [TT_h, Total];
    Contact_h = [Contact_h robot_hard.toe_contact];
    Collision_h = [Collision_h robot_hard.toe_collision];
    Contact_Point_h = [Contact_Point_h robot_hard.contact_point];
    x = x_next;
    t_h = robot_hard.time;
end
%%
figure(1);
for i=1:5
    subplot(5,1,i);
    plot(Tax_s, X_s(i,:), 'Color','k', 'LineWidth', 1.0);
    hold on;
    plot(Tax_h, X_h(i,:), 'Color','r', 'LineWidth', 1.0);
    grid on;
end
% subplot(7,1,6);
% stairs(Tax_s, Contact_s, 'Color','k', 'LineWidth', 1.0);
% hold on;
% stairs(Tax_h, Contact_h, 'Color','r', 'LineWidth', 1.0);
% subplot(7,1,7);
% stairs(Tax_s, Collision_s, 'Color','k', 'LineWidth', 1.0);
% hold on;
% stairs(Tax_h, Collision_h, 'Color','r', 'LineWidth', 1.0);
%%
figure(45);
plot(Tax_s , Contact_Point_s, 'k', 'LineWidth', 1.0); hold on;
plot(Tax_h , Contact_Point_h, 'r', 'LineWidth', 1.0);
%%
figure(3);
plot(Tax_s, TT_s, 'r', 'LineWidth', 1.0); hold on;
plot(Tax_h, TT_h, 'k', 'LineWidth', 1.0);
title("Energy");