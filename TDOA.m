BaseStationNum = 8;
% 基站位置
% BaseStationPosition = [0, 0, 11; 0, 20, 12; 0, 40, 9; 40, 0, 11; 40, 20, 8; 40, 40, 10];
BaseStationPosition = [0, 20, 0.1; 40, 20, 0; 20, 0, 0.2; 20, 40, 0.15; 0, 0, 5.1; 0, 40, 4.9; 40, 0, 5; 40, 40, 5.05];
% 生成终端位置及状态
TerminalPosition = [5, 5, 1];

[tspan, r, v, acc_inertial] = generateTerminalModel(step, TerminalPosition);

N = length(r);

range = generateRange(N, BaseStationNum, BaseStationPosition, r);
r_chan = TDOASolve(range, N, BaseStationNum, BaseStationPosition);

figure(1);
plot3(r(:, 1), r(:, 2), r(:, 3));


figure(2);
plot3(r_chan(:, 1), r_chan(:, 2), r_chan(:, 3));
