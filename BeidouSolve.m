function [r_beidouTOA] = BeidouSolve(r)

r_beidou=zeros(5001,3);

for i=1:5001
 num = 4;  %可以认为是基站或卫星数，随机做4个
 loc = r(i,:);                        %实际坐标位置
 sat = 100*rand(num,3);                  %基站或卫星二维坐标
 dist = loc - sat + 1.8*rand(num,3);     %加20厘米测距误差
 dist1 = dist(:,1).^2 + dist(:,2).^2+dist(:,3).^2;
 R = sat(:,1).^2 + sat(:,2).^2+sat(:,3).^2;
 X = [-2*sat ones(length(sat),1)];
 Y = dist1 - R;
 C = inv(X'*X)*X'*Y;
 r_beidou(i,:)=C(1:3)';%计算结果
end

 r_beidouTOA=rmoutliers(r_beidou);

end

