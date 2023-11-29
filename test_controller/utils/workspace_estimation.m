%% Hand downward
clear
clc

tl = tiledlayout(2,2);
title(tl, "Workspace estimation with right hand as end-effector")
colorMap1 = [ones(8,1), zeros(8,2)];
colorMap2 = [linspace(0,1,50)',linspace(1,0,50)',zeros(50,1)];
colorMap = [colorMap2; colorMap1];
%% Hand lw pos
set_pose_lw = readtable("poses_ws_1e-4_distance.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_distance.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};

position_error_lw = sqrt((set_pose_lw.pos_x - current_pose_lw.pos_x).^2 + ...
    (set_pose_lw.pos_y - current_pose_lw.pos_y).^2 + ...
    (set_pose_lw.pos_z - current_pose_lw.pos_z).^2);

position_error_lw(position_error_lw > 0.04) = 0.04;

nexttile
s2 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], position_error_lw, 'filled');
s2.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Position estimation with hand leftward oriented")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Hand dw pos
set_pose_lw = readtable("poses_ws_1e-4_distance_dw.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_distance_dw.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};

position_error_lw = sqrt((set_pose_lw.pos_x - current_pose_lw.pos_x).^2 + ...
    (set_pose_lw.pos_y - current_pose_lw.pos_y).^2 + ...
    (set_pose_lw.pos_z - current_pose_lw.pos_z).^2);

position_error_lw(position_error_lw > 0.04) = 0.04;

nexttile
s2 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], position_error_lw, 'filled');
s2.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Position estimation with hand downward oriented")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Hand lw orientation
set_pose_lw = readtable("poses_ws_1e-4_distance.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_distance.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));
angular_distance_lw(angular_distance_lw > 20) = 20;

%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Orientation estimation with hand leftward oriented")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% Hand dw orientation
set_pose_lw = readtable("poses_ws_1e-4_distance_dw.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_distance_dw.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));
angular_distance_lw(angular_distance_lw > 20) = 20;

%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Orientation estimation with hand downward oriented")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")