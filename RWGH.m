function [r] = RWGH(BSN, BSP, range)
% 对距离观测值进行残差加权解算
% 输入
% range是各历元各基站到终端的伪距测量值，1*BSN
% BSN是基站（观测量）数量
% BSP是各基站位置
% 输出r是各历元TOA解算的位置，n*3


m = 0;
rk = zeros(100, 3);
E = zeros(1, 100);
for j1 = 1 : BSN - 3
    for j2 = j1 + 1 : BSN - 2
        for j3 = j2 + 1 : BSN - 1
            for j4 = j3 + 1 : BSN
                m = m + 1;
                range_k = [range(j1), range(j2), range(j3), range(j4)];
                BSP_ = [BSP(j1, :); BSP(j2, :); BSP(j3, :); BSP(j4, :)];
                rk(m, :) = toaLS(4, BSP_, range_k');
                % 所有的残差平方和
                E(m) = 0;
                for k = 2 : BSN
                     E(m) = E(m) + dot(BSP(k, :) - rk(m, :), BSP(k, :) - rk(m, :));
                end
            end
        end
    end
end
rk = rk(1:m, :);
E = E(1:m);
r = (E.^(-1)) * rk / sum(E.^(-1));

end