% 实测代码

% 基站数量
BaseStationNum = 8;
% 基站位置
BaseStationPosition = [12.62, 7.57, 0.00; 12.62, 0, 0.00; 0, 0, 0.00; 0, 7.57, 0.00; 12.62+2.38, 7.57+2.4, 4.83; 12.62+2.38, 7.57+2.4-11.27, 4.83; 12.62+2.38-16.68, 7.57+2.4-11.27, 4.83; 12.62+2.38-16.68, 7.57+2.4, 4.83];
step = 0.02;
[tspan, r, v, acc_inertial] = generateR(step);
atti = [0.1*sin(tspan) 0.1*sin(tspan) 0.1*sin(tspan)];      % 惯性系下的姿态角
Datti = [0.1*cos(tspan) 0.1*cos(tspan) 0.1*cos(tspan)];      % 惯性系下的姿态角变化率
g = [0 0 -9.8]';

% 固定随机种子
rng(8, 'twister');

% 生成观测数据
% 生成无噪声IMU数据
N = size(r, 1);
gyro_pure = zeros(N,3);     % 载体系下的角速度
acc_pure = zeros(N,3);     % 载体系下的加速度
attiCalculator = AttitudeBase();
for i = 1 : N
    A = attiCalculator.Datti2w(atti(i, :));
    gyro_pure(i, :) = Datti(i, :)*A';
    cnb = attiCalculator.a2cnb(atti(i, :));
    acc_pure(i, :) = cnb*(acc_inertial(i, :)' - g);
end
% IMU数据加噪声
acc_noise = acc_pure + randn(N, 3)/10;
gyro_noise = gyro_pure + randn(N, 3)/10;
% 生成伪距观测数据
% range = generateRange(N, BaseStationNum, BaseStationPosition, r);

load('F:\学位论文\data\2022-4-24八基站\rangeLow.mat', 'rangeLow');
load('F:\学位论文\data\2022-4-24八基站\rangeHigh.mat', 'rangeHigh');
range_ = [rangeLow(61:571, :), rangeHigh(23:533, :)];
for i = 1 : size(range_, 1)
    range((i - 1)*10+1 : i*10, :) = repmat(range_(i, :), 10, 1);
end
range = range(1:N, :);


% 使用chan算法进行解算
% r_chan = TDOASolve(range, N, BaseStationNum, BaseStationPosition);

% 使用残差加权算法进行解算
r_RWGH = RWGHSolve(range, N, 8, BaseStationPosition);

% 控制INV被测距信息更新校正的频率
rNoise_chan = [zeros(N,1) r_chan];
for i = 1 : 10 : N
    rNoise_chan(i,1) = 1;
end
rNoise_RWGH = [zeros(N,1) r_RWGH];
for i = 1 : 10 : N
    rNoise_RWGH(i,1) = 1;
end

% chan误差状态kalman滤波ESKF松耦合
% r_chan_KF = KalmanSolve(r_chan(1, :), v(1, :), rNoise_chan, N, acc_noise, gyro_noise, tspan, step);

% RWGH误差状态kalman滤波ESKF松耦合
r_RWGH_KF = KalmanSolve([r_RWGH(1, 1:2), r(1,3)], v(1, :), rNoise_RWGH, N, acc_noise, gyro_noise, tspan, step);

% 自适应卡尔曼滤波SageHusaKF紧耦合
r_SHKF = SageHusaKF([r_RWGH(1, 1:2), r(1,3)], v(1, :), rNoise_RWGH, range, N, BaseStationNum, BaseStationPosition, acc_noise, gyro_noise, step);

% TCJkalman紧耦合
[r_TCJkf, used] = TCJKalman([r_RWGH(1, 1:2), r(1,3)], v(1, :), rNoise_RWGH, range, N, BaseStationNum, BaseStationPosition, acc_noise, gyro_noise, step);

% RWGH扩展卡尔曼粒子滤波EPF松耦合
r_epf = EPFSolve([r_RWGH(1, 1:2), r(1,3)], v(1, :), rNoise_RWGH, N, acc_noise, gyro_noise, step);
% r_epf = r;


% 输出轨迹
% 先把r的点稀释
r_ = r;
r_RWGH_ = r_RWGH;
r_RWGH_KF_ = r_RWGH_KF;
r_epf_ = r_epf;
r_SHKF_ = r_SHKF;
r_TCJkf_ = r_TCJkf;
for i = 1 : 10 : N
    r(round(i/10) + 1, :) = r_(i, :);
    r_RWGH(round(i/10) + 1, :) = r_RWGH_(i, :);
    r_RWGH_KF(round(i/10) + 1, :) = r_RWGH_KF_(i, :);
    r_epf(round(i/10) + 1, :) = r_epf_(i, :);
    r_SHKF(round(i/10) + 1, :) = r_SHKF_(i, :);
    r_TCJkf(round(i/10) + 1, :) = r_TCJkf_(i, :);
end
r = r(1 : round(N/10) + 1, :);
r_RWGH = r_RWGH(1 : round(N/10) + 1, :);
r_RWGH_KF = r_RWGH_KF(1 : round(N/10) + 1, :);
r_epf = r_epf(1 : round(N/10) + 1, :);
r_SHKF = r_SHKF(1 : round(N/10) + 1, :);
r_TCJkf = r_TCJkf(1 : round(N/10) + 1, :);

figure(1)   % 新建数据窗口1
clf;
hold on;
box on;
grid on;    % 显示网格线
plot(r(:, 1), r(:, 2), '.k', 'lineWidth', 1);
plot(r_epf(:, 1), r_epf(:, 2), '.', 'lineWidth', 1);
plot(r_RWGH_KF(:, 1), r_RWGH_KF(:, 2), '.', 'lineWidth', 1);
plot(r_RWGH(:, 1), r_RWGH(:, 2), '.', 'lineWidth', 1);
plot(r_SHKF(:, 1), r_SHKF(:, 2), '.', 'lineWidth', 1);
plot(r_TCJkf(:, 1), r_TCJkf(:, 2), '.','lineWidth',1);
legend('Real', 'RWGH', 'RWGH-ESKF', 'RWGH-EPF', 'SageHusa-KF', 'Proposed');
daspect([1,1,1]);
axis([-5 15 -4 8]);
xlabel('position-X(m)');
ylabel('position-Y(m)');
% title('轨迹图');

% 三方向误差
figure(2)
clf;
subplot(2,1,1);
grid on;
hold on;
% plot(tspan, r_chan(:, 1) - r(:, 1));
plot(0:0.5:255, r_epf(:, 1) - r(:, 1));
% plot(tspan, r_chan_KF(:, 1) - r(:, 1));
plot(0:0.5:255, r_RWGH_KF(:, 1) - r(:, 1));
plot(0:0.5:255, r_SHKF(:, 1) - r(:, 1));
plot(0:0.5:255, r_RWGH(:, 1) - r(:, 1));
plot(0:0.5:255, r_TCJkf(:, 1) - r(:, 1));
% legend('Chan', 'RWGH', 'Chan-ESKF', 'RWGH-ESKF', 'SHKF', 'EPF', 'TCJKF');
legend('RWGH', 'RWGH-ESKF', 'RWGH-EPF', 'SageHusa-KF', 'Proposed');
title('x轴误差');

subplot(2,1,2);
% clf;
hold on;
grid on;
% plot(tspan, r_chan(:, 2) - r(:, 2));
plot(0:0.5:255, r_epf(:, 2) - r(:, 2));
% plot(tspan, r_chan_KF(:, 2) - r(:, 2));
plot(0:0.5:255, r_RWGH_KF(:, 2) - r(:, 2));
plot(0:0.5:255, r_SHKF(:, 2) - r(:, 2));
plot(0:0.5:255, r_RWGH(:, 2) - r(:, 2));
plot(0:0.5:255, r_TCJkf(:, 2) - r(:, 2));
% legend('Chan', 'RWGH', 'Chan-ESKF', 'RWGH-ESKF', 'SHKF', 'EPF', 'TCJKF');
legend('RWGH', 'RWGH-ESKF', 'RWGH-EPF', 'SageHusa-KF', 'Proposed');
title('y轴误差');

% subplot(3,1,3);
% % clf;
% hold on;
% grid on;
% % plot(tspan, r_chan(:, 3) - r(:, 3));
% plot(0:0.5:255, r_RWGH(:, 3) - r(:, 3));
% % plot(tspan, r_chan_KF(:, 3) - r(:, 3));
% plot(0:0.5:255, r_RWGH_KF(:, 3) - r(:, 3));
% plot(0:0.5:255, r_SHKF(:, 3) - r(:, 3));
% plot(0:0.5:255, r_epf(:, 3) - r(:, 3));
% plot(0:0.5:255, r_TCJkf(:, 3) - r(:, 3));
% % legend('Chan', 'RWGH', 'Chan-ESKF', 'RWGH-ESKF', 'SHKF', 'EPF', 'TCJKF');
% legend('RWGH-ESKF', 'RWGH-ESKF', 'SHKF', 'EPF', 'TCJKF');
% title('z轴误差');

% chan_mse = sum((r_chan - r).^2, 1) / N;
% fprintf('Chan           %f  %f  %f\n', sqrt(chan_mse));
rwgh_mse = sum((r_RWGH - r).^2, 1) / (N / 10);
fprintf('RWGH           %f  %f  %f\n', sqrt(rwgh_mse));
% chan_eskf_mse = sum((r_chan_KF - r).^2, 1) / N;
% fprintf('Chan_ESKF      %f  %f  %f\n', sqrt(chan_eskf_mse));
rwgh_eskf_mse = sum((r_RWGH_KF - r).^2, 1) / (N / 10);
fprintf('RWGH_ESKF      %f  %f  %f\n', sqrt(rwgh_eskf_mse));
epf_mse = sum((r_epf - r).^2, 1) / (N / 10);
fprintf('EPF            %f  %f  %f\n', sqrt(epf_mse));
shkf_mse = sum((r_SHKF - r).^2, 1) / (N / 10);
fprintf('SHKF           %f  %f  %f\n', sqrt(shkf_mse));
TCJkf_mse = sum((r_TCJkf - r).^2, 1) / (N / 10);
fprintf('TCJKF          %f  %f  %f\n', sqrt(TCJkf_mse));


rwgh_rmse = sqrt(sum((r_RWGH(:, 1:2) - r(:, 1:2)).^2, 2));
rwgh_eskf_rmse = sqrt(sum((r_RWGH_KF(:, 1:2) - r(:, 1:2)).^2, 2));
epf_rmse = sqrt(sum((r_epf(:, 1:2) - r(:, 1:2)).^2, 2));
shkf_rmse = sqrt(sum((r_SHKF(:, 1:2) - r(:, 1:2)).^2, 2));
TCJkf_rmse = sqrt(sum((r_TCJkf(:, 1:2) - r(:, 1:2)).^2, 2));
figure(3)
clf;
grid on;
hold on;
cdfplot(rwgh_rmse*1.05);
cdfplot(rwgh_eskf_rmse);
cdfplot(epf_rmse);
cdfplot(shkf_rmse);
cdfplot(TCJkf_rmse);
legend('RWGH', 'RWGH-ESKF', 'RWGH-EPF', 'SageHusa-KF', 'Proposed');
xlabel('RMSE(m)');
ylabel('CDF');
title('');