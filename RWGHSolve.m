function [r] = RWGHSolve(range, N, BSN, BSP)
% 对距离观测值进行TOA解算
% 输入
% range是各历元各基站到终端的伪距测量值，N*BSN
% N是历元数量
% BSN是基站（观测量）数量
% BSP是各基站位置
% 输出r是各历元TOA解算的位置，n*3

r = zeros(N, 3);
for i = 1 : N
    r(i, :) = RWGH(BSN, BSP, range(i, :));
end

end