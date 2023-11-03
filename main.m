%生成数据
[r_RWGH,v,rNoise_RWGH, N, acc_noise, gyro_noise, tspan, step,range,BaseStationNum, BaseStationPosition,r] = generatedata();

% 自适应卡尔曼滤波紧耦合
r_Tight = KalmanSolve(r_RWGH(1, :), v(1, :), rNoise_RWGH, N, acc_noise, gyro_noise, tspan, step);

% RWGH误差状态kalman滤波松耦合
r_Loose = SageHusaKF(r_RWGH(1, :), v(1, :), rNoise_RWGH, range, N, BaseStationNum, BaseStationPosition, acc_noise, gyro_noise, step);

%北斗单源定位解算
r_BeidouSolve=BeidouSolve(r);

%画出CDF图
CDF_plot(r_BeidouSolve,r_Loose,r_Tight,r);





