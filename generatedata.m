function [r_RWGH,v,rNoise_RWGH, N, acc_noise, gyro_noise, tspan, step,range,BaseStationNum, BaseStationPosition,r] = generatedata()
% 基站数量
BaseStationNum= 8;

% 基站位置
BaseStationPosition=[0, 20, 0.1; 40, 20, 0; 20, 0, 0.2; 20, 40, 0.15; 0, 0, 5.1; 0, 40, 4.9; 40, 0, 5; 40, 40, 5.05];

% 生成终端位置及状态
TerminalPosition = [5, 5, 1];
step = 0.01;
[tspan, r, v, acc_inertial] = generateTerminalModel(step, TerminalPosition);
atti = [0.1*sin(tspan) 0.1*sin(tspan) 0.1*sin(tspan)];      % 惯性系下的姿态角
Datti = [0.1*cos(tspan) 0.1*cos(tspan) 0.1*cos(tspan)];      % 惯性系下的姿态角变化率
g = [0 0 -9.8]';

% 固定随机种子
rng(6, 'twister');

% 生成观测数据
% 生成无噪声IMU数据
N = length(r);
gyro_pure = zeros(N,3);     % 载体系下的角速度
acc_pure = zeros(N,3);     % 载体系下的加速度
attiCalculator = AttitudeBase();
for i = 1 : N
    A = attiCalculator.Datti2w(atti(i, :));%惯性系下的姿态角速率变成载体系的角速度的转化矩阵
    gyro_pure(i, :) = Datti(i, :)*A';   %求载体系角速度
    cnb = attiCalculator.a2cnb(atti(i, :));%惯性系到载体系的变换矩阵
    acc_pure(i, :) = cnb*(acc_inertial(i, :)' - g);%载体系下的加速度
end
% IMU数据加噪声
acc_noise = acc_pure + randn(N, 3);
gyro_noise = gyro_pure + randn(N, 3);
% 生成伪距观测数据
range = generateRange(N, BaseStationNum, BaseStationPosition, r); %生成终端真实位置与基站位置加了噪声的伪距



% 使用残差加权算法进行解算
r_RWGH = RWGHSolve(range, N, BaseStationNum, BaseStationPosition);



% 控制INV被测距信息更新校正的频率

rNoise_RWGH = [zeros(N,1) r_RWGH];
for i = 1 : 10 : N
    rNoise_RWGH(i,1) = 1;
end
end

