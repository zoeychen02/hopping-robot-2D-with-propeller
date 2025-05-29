% Stance phase controller
function [tau_hip, f_leg, f_fp, f_hp] = stance_controller(x,mtotal,g,h_desired,v_desired)

    %% define state
    robot_pos = [x(1);x(2)];
    robot_orient = [x(3);x(4)];
    robot_vel = [x(6);x(7)];
    robot_angvel = [x(8);x(9)];
    
    %% Physical limits
    k_spring = 100; % spring constant
    Kp_leg = 100; % leg P gain
    Kd_leg = 8; % leg D gain
    k_e = 14; % desire e gain
    kp1 = 3000; % hip torque P gain
    kd1 = 1; % hip torque D gain
    kp2 = 10; % hip torque P gain
    kd2 = 1; % hip torque D gain
     
%      if abs(robot_orient(2)) > 0.15 
%         tau_hip = kp1*(0-robot_orient(2))+kd1*(0-robot_angvel(2));
%      else
%         tau_hip = kp2*(0-robot_orient(2))+kd2*(0-robot_angvel(2));
%      end
%      
     tau_hip = 0;
     f_fp = 0;
     f_hp = 0;
    %% energy controller
     e_desired = k_e*(mtotal*g*(h_desired)+1/2*mtotal*v_desired^2); % desired e
     e = 1/2*mtotal*((robot_vel(1))^2 + (robot_vel(2))^2) + mtotal*g*(robot_pos(2)); %current e
     error =e_desired-e;
       
if error > 0
    l_leg_desired = sqrt(2 * error / k_spring) + 0.3;
    f_leg = Kp_leg * (l_leg_desired - x(5)) - Kd_leg * x(10);
else
    l_leg_desired = 0.3;
    f_leg = Kp_leg * (l_leg_desired - x(5)) - Kd_leg * x(10);
end

if f_leg >140
   f_leg = 140;

end
     
    %% Display results
%       disp(f_fp);
%       disp(f_hp);
%       disp(f_leg);
%       disp(x(6));


