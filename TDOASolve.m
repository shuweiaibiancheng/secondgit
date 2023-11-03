function [r] = TDOASolve(range, N, BaseStationNum, BaseStationPosition)
% 对距离观测值进行TDOA解算
% 输入
% range是各历元各基站到终端的伪距测量值，N*BaseStationNum
% N是历元数量
% BaseStationNum是基站（观测量）数量
% BaseStationPosition是各基站位置
% 输出r是各历元TDOA解算的位置，n*3

r = zeros(N, 3);
rangeI1 = zeros(N, BaseStationNum - 1);
for i = 1 : N
    for j = 1 : BaseStationNum - 1
        rangeI1(i, j) = range(i, j + 1) - range(i, 1);
    end
    if (i==2501)
        aaa = 1;
    end
    r(i, :) = Chan(BaseStationNum, BaseStationPosition, rangeI1(i, :));
end

end