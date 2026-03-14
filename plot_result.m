%% plot_result.m - IAPF 仿真结果可视化 (带经纬线渐变色障碍物版)
% 运行此脚本前，请确保 Simulink 仿真已完成，且工作区有 out 对象及初始化的参数

clc; close all;

% 1. 提取数据
% 假设 Simulink 输出对象为 out，变量名为 P_history (3 x N x T 维矩阵)
P_data = out.P_history; 
time = out.tout;        % 提取仿真时间向量
[~, num_agents, num_steps] = size(P_data);

% 定义颜色表，方便区分不同无人机
colors = lines(num_agents);

% =========================================================================
% ★ 核心美化：自定义渐变色池 (绿 -> 黄 -> 橙)
% =========================================================================
c_green  = [0.2, 0.8, 0.2];
c_yellow = [1.0, 0.9, 0.1];
c_orange = [1.0, 0.5, 0.0];
% 生成 64 阶平滑渐变色矩阵
custom_cmap = [linspace(c_green(1), c_yellow(1), 32)', linspace(c_green(2), c_yellow(2), 32)', linspace(c_green(3), c_yellow(3), 32)';
               linspace(c_yellow(1), c_orange(1), 32)', linspace(c_yellow(2), c_orange(2), 32)', linspace(c_yellow(3), c_orange(3), 32)'];

% =========================================================================
% 图 1: 三维轨迹图 (包含起点、终点、渐变色经纬度障碍物)
% =========================================================================
figure('Name', '3D Trajectories', 'Position', [100, 100, 800, 600]);
hold on; grid on; view(3);
title('IAPF 多无人机编队路径规划与避障三维轨迹', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 1.1 画目标点
plot3(q_target(1), q_target(2), q_target(3), 'p', 'MarkerSize', 18, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'DisplayName', 'Target');

% 1.2 画障碍物 (带有经纬度条纹与渐变色的三维球体)
[X_sph, Y_sph, Z_sph] = sphere(40); % 生成 40x40 球面网格，以显现经纬线
X_obs = obs_center(1) + obs_rho0 * X_sph;
Y_obs = obs_center(2) + obs_rho0 * Y_sph;
Z_obs = obs_center(3) + obs_rho0 * Z_sph;

% 使用 Z_obs 作为颜色映射矩阵 (CData)，并设置 EdgeColor 显示经纬线
surf(X_obs, Y_obs, Z_obs, Z_obs, ...
     'FaceAlpha', 0.6, 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5, ...
     'DisplayName', 'Obstacle (Safety Zone)');
colormap(gca, custom_cmap); % 赋予自定义的 绿-黄-橙 渐变色
 
% 1.3 画轨迹和起点
for i = 1:num_agents
    % 提取第 i 架飞机所有时间的三维坐标
    x_traj = squeeze(P_data(1, i, :));
    y_traj = squeeze(P_data(2, i, :));
    z_traj = squeeze(P_data(3, i, :));
    
    % 标出起点
    plot3(x_traj(1), y_traj(1), z_traj(1), 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', colors(i,:), 'Color', colors(i,:), 'HandleVisibility', 'off');
    
    % 画轨迹
    if i == 1
        plot3(x_traj, y_traj, z_traj, '-', 'Color', colors(i,:), 'LineWidth', 2.5, 'DisplayName', 'Leader');
    else
        plot3(x_traj, y_traj, z_traj, '--', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', ['Follower ', num2str(i)]);
    end
end
legend('Location', 'best', 'FontSize', 11);

% =========================================================================
% 图 2: 三轴编队跟踪误差曲线图 (按 X, Y, Z 轴分图)
% =========================================================================
figure('Name', 'Formation Tracking Errors (XYZ)', 'Position', [950, 100, 600, 800]);
sgtitle('所有跟随者相对领航者的编队跟踪误差', 'FontSize', 15, 'FontWeight', 'bold');

P_leader = squeeze(P_data(:, 1, :)); % 3 x T
axis_titles = {'X轴编队误差 (e_x)', 'Y轴编队误差 (e_y)', 'Z轴编队误差 (e_z)'};
y_labels = {'Error X (m)', 'Error Y (m)', 'Error Z (m)'};

for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on; grid on;
    title(axis_titles{axis_idx});
    ylabel(y_labels{axis_idx});
    if axis_idx == 3, xlabel('Time (s)'); end
    
    for i = 2:num_agents
        P_follower = squeeze(P_data(:, i, :)); % 3 x T
        error_xyz = P_follower - (P_leader + delta(:, i)); 
        plot(time, error_xyz(axis_idx, :), 'LineWidth', 1.5, ...
             'Color', colors(i,:), 'DisplayName', ['Follower ', num2str(i)]);
    end
    if axis_idx == 1, legend('Location', 'best', 'NumColumns', 2); end
end

% =========================================================================
% 图 3: 距离障碍物的距离曲线图 (防碰撞与穿模验证)
% =========================================================================
figure('Name', 'Distance to Obstacle', 'Position', [100, 400, 800, 300]);
hold on; grid on;
title('所有无人机距离障碍物中心的实时距离', 'FontSize', 14);
xlabel('Time (s)'); ylabel('Distance (m)');

yline(obs_rho0, 'r-', 'LineWidth', 2.5, 'DisplayName', '碰撞红线 (obs\_rho0)');

for i = 1:num_agents
    P_i = squeeze(P_data(:, i, :)); % 3 x T
    dist_to_obs = sqrt((P_i(1,:) - obs_center(1)).^2 + ...
                       (P_i(2,:) - obs_center(2)).^2 + ...
                       (P_i(3,:) - obs_center(3)).^2);
    if i == 1
        plot(time, dist_to_obs, '-', 'Color', colors(i,:), 'LineWidth', 2.5, 'DisplayName', 'Leader');
    else
        plot(time, dist_to_obs, '--', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', ['Follower ', num2str(i)]);
    end
end
legend('Location', 'bestoutside');
ylim([0, max(max(dist_to_obs))*1.1]);

% =========================================================================
% 图 4: 动态三维轨迹动画
% =========================================================================
figure('Name', '3D Dynamic Animation', 'Position', [200, 200, 900, 700]);
hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 1. 锁定三维坐标轴范围
min_x = min(min(P_data(1,:,:))) - 10; max_x = max(max(P_data(1,:,:))) + 10;
min_y = min(min(P_data(2,:,:))) - 10; max_y = max(max(P_data(2,:,:))) + 10;
min_z = min(min(P_data(3,:,:))) - 10; max_z = max(max(P_data(3,:,:))) + 10;
axis([min_x, max_x, min_y, max_y, min_z, max_z]);

% 2. 绘制静态背景 (目标点与带经纬线渐变色的障碍物)
plot3(q_target(1), q_target(2), q_target(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
surf(X_obs, Y_obs, Z_obs, Z_obs, ...
     'FaceAlpha', 0.6, 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
colormap(gca, custom_cmap); % 同样赋予动画中的障碍物渐变色

% 3. 初始化动画句柄
h_pos = zeros(1, num_agents);   
h_traj = zeros(1, num_agents);  
for i = 1:num_agents
    if i == 1
        h_pos(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        h_traj(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), '-', 'Color', colors(i,:), 'LineWidth', 2.5);
    else
        h_pos(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        h_traj(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
end

h_title = title('IAPF 动态过程 - Time: 0.00 s', 'FontSize', 16, 'FontWeight', 'bold');

% 4. 开始播放动画
frame_step = round(num_steps / 200); 
if frame_step < 1, frame_step = 1; end

for t = 1:frame_step:num_steps
    set(h_title, 'String', sprintf('IAPF 动态过程 - Time: %.2f s', time(t)));
    for i = 1:num_agents
        set(h_pos(i), 'XData', P_data(1,i,t), 'YData', P_data(2,i,t), 'ZData', P_data(3,i,t));
        set(h_traj(i), 'XData', squeeze(P_data(1,i,1:t)), 'YData', squeeze(P_data(2,i,1:t)), 'ZData', squeeze(P_data(3,i,1:t)));
    end
    drawnow; 
end

for i = 1:num_agents
    set(h_pos(i), 'XData', P_data(1,i,end), 'YData', P_data(2,i,end), 'ZData', P_data(3,i,end));
    set(h_traj(i), 'XData', squeeze(P_data(1,i,:)), 'YData', squeeze(P_data(2,i,:)), 'ZData', squeeze(P_data(3,i,:)));
end
set(h_title, 'String', sprintf('IAPF 动态过程 - 抵达终点 (Time: %.2f s)', time(end)));