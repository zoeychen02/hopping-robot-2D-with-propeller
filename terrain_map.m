function [x_m, z_m] = terrain_map(x)
%TERRAIN_MAP create global terrain map function (in inertial frame)
%   return the map point
    % sin wave terrain
    x_m = x;
    z_m = 0; % 0.2 * sin(0.8 * x) + 0.3;
    % stairs
%     if( 0<=x && x<=0.4)
%         z_m = 0.1;
%     elseif( 0.4 < x && x <= 0.8)
%         z_m = 0.2;
%     elseif ( 0.8 < x && x <=1.6)
%         z_m = 0.4;
%     else
%         z_m = 0.6;
%     end
    % add tangential direction vector
    % add normal direction vector
end

