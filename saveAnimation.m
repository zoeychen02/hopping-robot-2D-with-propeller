function [] = saveAnimation(T, X, rbt, name)
% Create video of system
% Turn off plot visibility temporarily during frame rendering
set(0,'DefaultFigureVisible','off');

% Create figure for frames
fig = figure;
ax = gca;
ax.NextPlot = 'ReplaceChildren';

% Compute number of frames
num_frame = length(T);

% Counter for frames processed
n = 1;

% Open video for writing
vid = VideoWriter(name, 'Uncompressed AVI');
vid.FrameRate = 20;
open(vid);

% Loop over points in time to process frames at
for k = 1:20:num_frame
    q = X(1:rbt.nq,k);
    dqdt = X(1+rbt.nq:end,k);
    kin_res = rbt.foward_kinematics(q, dqdt);
    % Plot the joint between cart and pole
    pos_body = kin_res.fk_body(1:3);
    pos_fp = kin_res.fk_fp(1:2);
    pos_hp = kin_res.fk_hp(1:2);
    pos_lc = kin_res.fk_lc(1:2);
    pos_toe = kin_res.fk_toe(1:2);
    xrange = -1:0.01:2;
    Ym = zeros(1,length(xrange));
    for i=1:length(xrange)
        [~, ym] = rbt.terrain_map(xrange(i));
        Ym(:,i) = ym;
    end
    p1=plot(xrange, Ym, 'k', 'LineWidth',1); hold on;
    p2=plot([pos_hp(1), pos_fp(1)], [pos_hp(2), pos_fp(2)],...
         'Color', 'r' ,'LineWidth', 2.0);
    hold on;
    p3=plot([pos_body(1), pos_lc(1)], [pos_body(2), pos_lc(2)],...
         'Color', 'g' ,'LineWidth', 4.0);
    p4=plot([pos_lc(1), pos_toe(1)], [pos_lc(2), pos_toe(2)],...
         'Color', 'k' ,'LineWidth', 2.0);
    hold on;
    rbt.draw_circle([pos_body(1), pos_body(2)], 0.02, 'k'); hold on;
    rbt.draw_circle([pos_fp(1), pos_fp(2)], 0.025, 'r'); hold on;
    rbt.draw_circle([pos_hp(1), pos_hp(2)], 0.025, 'b'); hold on;
    rbt.draw_circle([pos_lc(1), pos_lc(2)], 0.01, 'm'); hold on;
    p1.Color(4) = 1;
    p2.Color(4) = 1;
    p3.Color(4) = 1;
    p4.Color(4) = 1;
    xlim([pos_body(1)-1, pos_body(1)+1]);
    ylim([0, 2.5]);
    axis equal; 
    hold off;
    
    % Apply blank background
    axis off;
    
    % Record the frame and store in the structure
    F = getframe(fig);
    
    % Write frame to video
    writeVideo(vid, F);
    
    % Display progress to command line
    if mod(k, 100) ==0 
        fprintf("rendering video frame %d out of %d...\n", k, num_frame);
    end
    % Increment frame counter
    n = n + 1;
end

% Close the video
close(vid);
end