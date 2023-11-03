function [r] = KalmanSolve(r0, v0, gps_noise, N, acc_noise, gyro_noise, tspan, step)
% 对TDOA测量值进行Kalman滤波
% 输入
% r0是初始位置
% v0是初始速度
% gps_pure是未经滤波的位置解算数据
% N是历元数量
% acc_noise是加速度计观测值
% gyro_noise是角速度计观测值
% tspan是各历元时刻
% step是历元间隔
% 输出r是滤波后的各历元位置

state0 = zeros(16, 1);
state0(1:3) = r0;
state0(4:6) = v0;
state0(7) = 1;
errorstate0 = zeros(15, 1);%误差初始状态赋值
Cov=[0.01*ones(3, 1); zeros(3, 1); 0.01*ones(3, 1); zeros(3, 1)];
Qc0=diag(Cov);%初始噪声方差
Rc0=diag([0.5, 0.5, 2]);%GPS测量噪声误差方差
% % 控制INV被测距信息更新校正的频率
% gps_noise=[zeros(N,1) gps_pure];
% for i=1:10:N
%     gps_noise(i,1)=1;
% end
%可以改变量使输入的惯性元件的数据带噪声或者不带
ins = InsSolver(Qc0,Rc0);
[state, errorstate] = ins.imu2state(acc_noise, gyro_noise, gps_noise, state0, errorstate0, tspan, step, 0);
r = state(:, 1:3);

end


