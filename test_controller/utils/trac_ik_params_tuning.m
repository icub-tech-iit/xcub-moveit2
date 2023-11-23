%% HAND LEFTWARD
clear
clc
tl = tiledlayout(2,4);
title(tl, "Manipulation1 with right hand leftward")
colorMap1 = [ones(8,1), zeros(8,2)];
colorMap2 = [linspace(0,1,50)',linspace(1,0,50)',zeros(50,1)];
colorMap = [colorMap2; colorMap1];

%% 1e^-3 pos
set_pose_lw = readtable("poses_ws_1e-3_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-3_manipulation1.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};

position_error_lw = sqrt((set_pose_lw.pos_x - current_pose_lw.pos_x).^2 + ...
    (set_pose_lw.pos_y - current_pose_lw.pos_y).^2 + ...
    (set_pose_lw.pos_z - current_pose_lw.pos_z).^2);

position_error_lw(position_error_lw > 0.04) = 0.04;

nexttile
s1 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], position_error_lw, 'filled');
s1.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-3} position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% 1e^-4 pos
set_pose_lw = readtable("poses_ws_1e-4_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation1.txt");
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
title("Tollerance 1e^{-4} position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% 1e^-5 pos
set_pose_lw = readtable("poses_ws_lw.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_lw.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};

position_error_lw = sqrt((set_pose_lw.pos_x - current_pose_lw.pos_x).^2 + ...
    (set_pose_lw.pos_y - current_pose_lw.pos_y).^2 + ...
    (set_pose_lw.pos_z - current_pose_lw.pos_z).^2);

position_error_lw(position_error_lw > 0.04) = 0.04;

nexttile
s3 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], position_error_lw, 'filled');
s3.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-5} position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% 1e^-6 pos
set_pose_lw = readtable("poses_ws_1e-6_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-6_manipulation1.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};

position_error_lw = sqrt((set_pose_lw.pos_x - current_pose_lw.pos_x).^2 + ...
    (set_pose_lw.pos_y - current_pose_lw.pos_y).^2 + ...
    (set_pose_lw.pos_z - current_pose_lw.pos_z).^2);

position_error_lw(position_error_lw > 0.04) = 0.04;

nexttile
s4 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], position_error_lw, 'filled');
s4.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-6} position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% 1e-3 orientation
set_pose_lw = readtable("poses_ws_1e-3_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-3_manipulation1.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));

%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-3} orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% 1e-4 orientation
set_pose_lw = readtable("poses_ws_1e-4_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation1.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));

%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-4} orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% 1e-5 orientation
set_pose_lw = readtable("poses_ws_lw.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_lw.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));

%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-5} orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% 1e-6 orientation
set_pose_lw = readtable("poses_ws_1e-6_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-6_manipulation1.txt");
current_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w'};
set_quaternion_lw = [set_pose_lw.or_w, set_pose_lw.or_x, set_pose_lw.or_y, set_pose_lw.or_z];
quat_id_lw = quaternion(set_quaternion_lw);
current_quaterion_lw = [current_pose_lw.or_w, current_pose_lw.or_x, current_pose_lw.or_y, current_pose_lw.or_z];
quat_real_lw = quaternion(current_quaterion_lw);

%(w,x,y,z) convention
angular_distance_lw = rad2deg(dist(quat_id_lw, quat_real_lw));
angular_distance_lw(angular_distance_lw > 26) = 26;
%plot
nexttile
s5 = scatter3(set_pose_lw.pos_x, set_pose_lw.pos_y, set_pose_lw.pos_z, [], angular_distance_lw, 'filled');
s5.SizeData = 50;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Tollerance 1e^{-6} orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% Change solve type
figure(2)
tl1 = tiledlayout(2,4);
title(tl1, "Solve type with tollerance 1e^{-4} with right hand leftward")

%% Distance pos
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
title("Distance position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Manipulation1 pos
set_pose_lw = readtable("poses_ws_1e-4_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation1.txt");
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
title("Manipulation1 position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Manipulation2 pos
set_pose_lw = readtable("poses_ws_1e-4_manipulation2.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation2.txt");
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
title("Manipulation2 position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Speed pos
set_pose_lw = readtable("poses_ws_1e-4_speed.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_speed.txt");
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
title("Speed position")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
% colormap(cool);
cb2 = colorbar;
xlabel(cb2, "Position error [m]")

%% Distance or
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
title("Distance orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% Manip1 or
set_pose_lw = readtable("poses_ws_1e-4_manipulation1.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation1.txt");
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
title("Manipulation1 orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% Manip2 or
set_pose_lw = readtable("poses_ws_1e-4_manipulation2.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_manipulation2.txt");
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
title("Manipulation2 orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")

%% Speed or
set_pose_lw = readtable("poses_ws_1e-4_speed.txt");
set_pose_lw.Properties.VariableNames = {'pos_x', 'pos_y', 'pos_z', 'or_x', 'or_y', 'or_z', 'or_w', 'success', 'percentage'};
current_pose_lw = readtable("current_data_1e-4_speed.txt");
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
title("Speed orientation")
xlim([-0.3, -0.1])
ylim([-0.3, 0.4])
zlim([-0.3, 0.3])
daspect([5 5 5])
colormap(colorMap);
cb2 = colorbar;
xlabel(cb2, "Angular error [deg]")
