function [tspan, r, v, acc_inertial] = generateTerminalModel(step, TerminalPosition)
% 生成终端的真实状态信息
% 输入
% step是历元间隔
% TerminalPosition是终端运动初始位置
% 输出
% tspan是各历元时刻
% r是各历元终端真实位置
% v是各历元终端真实速度
% acc_inertial是各历元终端真实加速度

start_time = 0;
end_time = 50;
tspan = (start_time : step : end_time)';
N = length(tspan);
r = zeros(N, 3);
v = zeros(N, 3);
acc_inertial = zeros(N, 3);
for i = 1 : N
    if (tspan(i) < 10)
        r(i, :) = [0, 1 * tspan(i), 0];
        v(i, :) = [0, 1, 0];
        acc_inertial(i, :) = [0, 0, 0];
    end
    if (tspan(i) == 10 - step)
         acc_inertial(i, :) = [0, (pi - 1) / step, 0.1 / step];
    end
    if (tspan(i) >= 10) && (tspan(i) < 40)
        t = tspan(i) - 10;
        r(i, :) = [-15 * cos(t * pi / 30), 15 * sin(t * pi / 30), 0.1 * t];
        r(i, :) = r(i, :) + [15, 10, 0];
        v(i, :) = [0.5 * pi * sin(t * pi / 30), 0.5 * pi * cos(t * pi / 30), 0.1];
        acc_inertial(i, :) = [pi^2 / 60 * cos(t * pi / 30), -pi^2 / 60 * sin(t * pi / 30), 0];
    end
    if (tspan(i) == 40 - step)
         acc_inertial(i, :) = [0, -(pi - 1) / step, -0.1 / step];
    end
    if (tspan(i) >= 40) && (tspan(i) <= 50)
        t = tspan(i) - 40;
        r(i, :) = [0, -1 * t, 0];
        r(i, :) = r(i, :) + [30, 10, 3];
        v(i, :) = [0, -1, 0];
        acc_inertial(i, :) = [0, 0, 0];
    end
    r(i, :) = r(i, :) + TerminalPosition;
end


%验证v
r1 = zeros(N, 3);
r1(1, :) = r(1, :);
for i = 2 : N
    r1(i, :) = r1(i - 1, :) + v(i - 1, :) * step;
end
plot3(r1(:,1),r1(:,2),r1(:,3));
axis([0, 40, 0, 40, 0, 10]);

