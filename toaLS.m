function [r] = toaLS(BSN, BSP, range)
% BSN为基站数量
% BSP为基站坐标，大小为[BSN, 3]
% range为TOA输入，大小为[BSN]，基站-终端的距离
% x, y, z为计算出的终端坐标
% R为xyz范数

A = [-2*BSP, ones(BSN, 1)];
B = range.^2 - sum(BSP.* BSP, 2);
X = pinv(A' * A) * A' * B;
r = X(1:3)';