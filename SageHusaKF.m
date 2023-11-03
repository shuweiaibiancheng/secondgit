function [r] = SageHusaKF(r0, v0, rNoise, rangeNoise, N, BSN, BSP, acc_noise, gyro_noise, step)
% 基于紧耦合的视距混合非视距环境定位
% 输入
% r0是初始位置
% v0是初始速度
% 伪距观测值
% rNoise是未经滤波的TDOA解算数据
% rangeNoise是各历元各基站到终端距离的观测数据
% N是历元数量
% BSN是基站数量
% BSP是各基站位置
% acc_noise是加速度计观测值
% gyro_noise是角速度计观测值
% step是历元间隔
% 输出r
% 算法输出的各历元位置

% 初始化
% 要维护的测距值
% range = zeros(1, BSN);
% for i = 1 : BSN
%     range(i) = sqrt(dot(BSP(i, :) - r0, BSP(i, :) - r0));
% end
r = zeros(N, 3);
r(1, :) = r0;
% 滤波状态初始化
state = zeros(1, 16);
state(1:3) = r0;
state(4:6) = v0;
state(7) = 1;
Cov = [0.01*ones(3, 1); zeros(3, 1); 0.01*ones(3, 1); zeros(3, 1)];
% 状态转移噪声
Qc0 = diag(Cov);
% 测量噪声，没用到
Rc0 = diag(0.01 * ones(1, BSN));
% 预测与更新的状态差值
errorstate = zeros(1, 15);
% 状态量协方差
P = zeros(15);
ins = InsSolver(Qc0, Rc0);
% 时间衰减因子
b = 0.95 + 1e-8;

  rangeI1 = zeros(N, BSN - 1);
  for j = 1 : N
            for k = 1 : BSN - 1
           rangeI1(j, k) = rangeNoise(j, k + 1) - rangeNoise(j, 1);  %观测到的TDOA
            end
       end

for i = 2 : N
    % 预测过程
    % 预测过程，生成先验
    % 生成状态转移信息
    % 惯导数据使用,四元数的imu动力学方程
    acc = acc_noise(i - 1, :);
    gyro = gyro_noise(i - 1, :);
    imudata = [acc'; gyro'];
    dy = ins.imuDynamics([], state', imudata);
    state = state + step * dy';
    predQ = ins.QuatNormalize(state(7:10));
    predCbn = ins.attiCalculator.quat2cnb(predQ);
    % 预测过程是非线性的，需要求离散化雅克比
    [Fd,Gc,Fc] = ins.getExponentMatFd(predCbn, step, acc - state(14:16), gyro - state(11:13));
    % 重置预测误差状态量
    errorstate = zeros(1, 15);ins
    % 预测偏差方差矩阵计算
    Qd = ins.getPredCovarianceMatQd(Gc, Fc, ins.Qc, step);
    P = Fd * P * Fd' + Qd;
    
    
    
    % 如果当前时刻有观测值，进入判决与kalman更新过程
    if (rNoise(i, 1) == 1)
        % 观测方程
%         for j = 1 : BSN   
%             range(j) = sqrt(dot(BSP(j, :) - state(1:3), BSP(j, :) - state(1:3)));
%         end

       %预测到的TDOA
       rangeYuce = zeros(N, BSN);
             for j = 1 : BSN
        rangeYuce(j) = sqrt(dot(state(1:3) - BSP(j, :), state(1 :3) - BSP(j, :)));
             end
           for k = 1: BSN-1 
               rangeYuceTDOA(k) = rangeYuce(k+1) - rangeYuce(1);
           end

           diftdoa =  rangeI1(i, :) - rangeYuceTDOA;
       
%         DifRange = rangeNoise(i, :) - range;
        
        % kalman更新
        % 观测矩阵 雅克比
        H = [];
        rangeUse = [];
        for j = 1 : BSN-1
            % 构建观测矩阵
             H = [H;((state(1:3) - BSP(j+1, :)) / rangeYuce(j+1)) - ((state(1:3) - BSP(1, :)) / rangeYuce(1)), zeros(1, 12)];
             %H是观测方程求导 TDOA = （state（1：3）-BSP（j+1）)^1/2 -(State(1:3)-BSP(1))^1/2
            rangeUse = [rangeUse, rangeI1(i, j) - rangeYuceTDOA(j)];
        end
        rangeUse = [rangeUse,0];
        H = [H;zeros(1,15)];
        K = P * H' * (H * P * H' + ins.Rc)^-1;        %kalman增益
        errorstate = errorstate + (rangeUse - errorstate * H') * K';%后验偏差状态估计
        P = P - K * H * P;                  %后验偏差误差方差阵
        
        d = (1 - b) / (1 - b^i);
        % 视距下还是有用的
%         ins.Rc = (1 - d) * ins.Rc + d * (DifRange' * DifRange - H * P * H');
        
        state(1:6) = state(1:6) + errorstate(1:6);
        state(7:10) = ins.attiCalculator.QuatMulitMat(state(7:10), [1, 0.5 * errorstate(7:9)]');
        state(7:10) = ins.QuatNormalize(state(7:10));
        state(11:16) = state(11:16) + errorstate(10:15);    %这个是否需要进行偏差矫正以后视情况而定，涉及姿态的就不用了
    end
    
    r(i, :) = state(1:3);
end
    