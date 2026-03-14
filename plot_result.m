%% plot_result.m - IAPF 仿真结果可视化
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
% 图 1: 三维轨迹图 (包含起点、终点、半透明障碍物)
% =========================================================================
figure('Name', '3D Trajectories', 'Position', [100, 100, 800, 600]);
hold on; grid on; view(3);
title('IAPF 多无人机编队路径规划与避障三维轨迹', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 1.1 画目标点
plot3(q_target(1), q_target(2), q_target(3), 'p', 'MarkerSize', 18, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'DisplayName', 'Target');

% 1.2 画障碍物 (半透明球体)
[X_sph, Y_sph, Z_sph] = sphere(50); % 生成球面网格
% 将标准球体缩放至警戒半径，并平移至障碍物中心
surf(obs_center(1) + obs_rho0 * X_sph, ...
     obs_center(2) + obs_rho0 * Y_sph, ...
     obs_center(3) + obs_rho0 * Z_sph, ...
     'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5], ...
     'DisplayName', 'Obstacle (Safety Zone)');
 
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

% 提取领航者轨迹作为参考基准
P_leader = squeeze(P_data(:, 1, :)); % 3 x T

% 定义各个子图的标题和 Y 轴标签
axis_titles = {'X轴编队误差 (e_x)', 'Y轴编队误差 (e_y)', 'Z轴编队误差 (e_z)'};
y_labels = {'Error X (m)', 'Error Y (m)', 'Error Z (m)'};

for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on; grid on;
    title(axis_titles{axis_idx});
    ylabel(y_labels{axis_idx});
    
    % 只有最下面的子图需要 X 轴标签
    if axis_idx == 3
        xlabel('Time (s)');
    end
    
    % 在当前轴 (X, Y 或 Z) 下，遍历画出所有跟随者的误差
    for i = 2:num_agents
        P_follower = squeeze(P_data(:, i, :)); % 3 x T
        
        % 计算三轴误差: 实际位置 - 期望位置 (Leader位置 + delta偏差)
        error_xyz = P_follower - (P_leader + delta(:, i)); 
        
        % 提取当前轴 (axis_idx) 的误差并画图
        plot(time, error_xyz(axis_idx, :), 'LineWidth', 1.5, ...
             'Color', colors(i,:), 'DisplayName', ['Follower ', num2str(i)]);
    end
    
    % 在 X 轴子图上显示图例 (排成两列更美观)
    if axis_idx == 1
        legend('Location', 'best', 'NumColumns', 2);
    end
end

% =========================================================================
% 图 3: 距离障碍物的距离曲线图 (防碰撞与穿模验证)
% =========================================================================
figure('Name', 'Distance to Obstacle', 'Position', [100, 750, 800, 300]);
hold on; grid on;
title('所有无人机距离障碍物中心的实时距离', 'FontSize', 14);
xlabel('Time (s)'); ylabel('Distance (m)');

% 画出危险红线 (障碍物的半径)
yline(obs_rho0, 'r-', 'LineWidth', 2.5, 'DisplayName', '碰撞红线 (obs\_rho0)');

for i = 1:num_agents
    P_i = squeeze(P_data(:, i, :)); % 3 x T
    
    % 计算实时欧氏距离
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
% 限制Y轴下限，让曲线不会跑到底下导致看不清红线
ylim([0, max(max(dist_to_obs))*1.1]);

% =========================================================================
% 图 4: 动态三维轨迹动画 (直观展示"领航者狂飙"与"跟随者掉队"现象)
% =========================================================================
% 提示：如果你之前用了 max_pull_dist 限幅，请先把它注释掉，还原巨大的起步误差来录制这个动画

figure('Name', '3D Dynamic Animation', 'Position', [200, 200, 900, 700]);
hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 1. 锁定三维坐标轴范围 (极其重要！)
% 如果不锁定，动画播放时坐标轴会跟着飞机不断放大缩小，让人眼花缭乱
min_x = min(min(P_data(1,:,:))) - 10; max_x = max(max(P_data(1,:,:))) + 10;
min_y = min(min(P_data(2,:,:))) - 10; max_y = max(max(P_data(2,:,:))) + 10;
min_z = min(min(P_data(3,:,:))) - 10; max_z = max(max(P_data(3,:,:))) + 10;
axis([min_x, max_x, min_y, max_y, min_z, max_z]);

% 2. 绘制静态背景 (目标点与半透明障碍物)
plot3(q_target(1), q_target(2), q_target(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
surf(obs_center(1) + obs_rho0 * X_sph, ...
     obs_center(2) + obs_rho0 * Y_sph, ...
     obs_center(3) + obs_rho0 * Z_sph, ...
     'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5]);

% 3. 初始化动画句柄 (飞机本体 和 飞行拖尾)
h_pos = zeros(1, num_agents);   % 飞机当前的圆点
h_traj = zeros(1, num_agents);  % 飞机身后的轨迹线

for i = 1:num_agents
    if i == 1
        % 领航者：大号实心圆点 + 粗实线拖尾
        h_pos(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        h_traj(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), '-', 'Color', colors(i,:), 'LineWidth', 2.5);
    else
        % 跟随者：普通圆点 + 虚线拖尾
        h_pos(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        h_traj(i) = plot3(P_data(1,i,1), P_data(2,i,1), P_data(3,i,1), '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
end

% 初始化动态标题
h_title = title('IAPF 动态过程 - Time: 0.00 s', 'FontSize', 16, 'FontWeight', 'bold');

% 4. 开始播放动画
% 为了防止渲染太慢，我们采用"跳帧播放"策略。比如每 40 步更新一次画面。
% 如果你的 dt=0.01, 80秒有8000步，跳40步意味着生成 200 帧动画，非常流畅。
frame_step = round(num_steps / 200); 
if frame_step < 1, frame_step = 1; end

for t = 1:frame_step:num_steps
    % 4.1 更新时间戳
    set(h_title, 'String', sprintf('IAPF 动态过程 - Time: %.2f s', time(t)));
    
    % 4.2 更新每架飞机的坐标
    for i = 1:num_agents
        % 更新飞机本体的位置 (XData, YData, ZData 只能是单一数值)
        set(h_pos(i), 'XData', P_data(1,i,t), ...
                      'YData', P_data(2,i,t), ...
                      'ZData', P_data(3,i,t));
                  
        % 更新拖尾轨迹 (把从 1 到 t 时刻的所有坐标塞进去)
        set(h_traj(i), 'XData', squeeze(P_data(1,i,1:t)), ...
                       'YData', squeeze(P_data(2,i,1:t)), ...
                       'ZData', squeeze(P_data(3,i,1:t)));
    end
    
    % 4.3 强制 MATLAB 立即刷新图像窗口
    drawnow; 
    
    % 如果你想让动画播放得慢一点，可以取消下面这行的注释
    % pause(0.01); 
end

% 确保最后一帧停在最终位置
for i = 1:num_agents
    set(h_pos(i), 'XData', P_data(1,i,end), 'YData', P_data(2,i,end), 'ZData', P_data(3,i,end));
    set(h_traj(i), 'XData', squeeze(P_data(1,i,:)), 'YData', squeeze(P_data(2,i,:)), 'ZData', squeeze(P_data(3,i,:)));
end
set(h_title, 'String', sprintf('IAPF 动态过程 - 抵达终点 (Time: %.2f s)', time(end)));