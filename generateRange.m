function [range] = generateRange(N, BSN, BSP, r)


% 生成与误差同一的伪距
range = zeros(N, BSN);
for i = 1 : N
    for j = 1 : BSN
         range(i, j) = normrnd(0, 0.8) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
%         range(i, j) =  sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
    end
end


% % 生成与基站个体有关的伪距
% range = zeros(N, BSN);
% sigma = 0.5 * rand(1, BSN);
% for i = 1 : N
%     for j = 1 : BSN
%         range(i, j) = normrnd(0, sigma(j)) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
%     end
% end


% % 生成带有非视距误差的伪距
% range = zeros(N, BSN);
% sigma = 0.1 + 0.2 * rand(1, BSN);
% NLOS = 1 + 2 * rand(1, BSN);
% NLOSB = [0, 0, 0, 0, 0, 0, 0, 0];
% for i = 1 : 1000
%     for j = 1 : BSN
%         range(i, j) = normrnd(0, sigma(j)) + NLOSB(j) * NLOS(j) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
%     end
% end
% NLOSB = [0, 1, 0, 0, 1, 0, 1, 0];
% % NLOSB = [0, 0, 0, 0, 0, 0, 0, 0];
% for i = 1001 : 3000
%     for j = 1 : BSN
%         range(i, j) = normrnd(0, sigma(j)) + NLOSB(j) * NLOS(j) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
%     end
% end
% % for i = 2001 : 3000
% %     for j = 1 : BSN - 1
% %         range(i, j) = normrnd(0, sigma(j)) + NLOS(j) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
% %     end
% %     for j = BSN : BSN
% %         range(i, j) = normrnd(0, sigma(j)) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
% %     end
% % end
% NLOSB = [0, 0, 1, 0, 0, 0, 0, 1];
% % NLOSB = [0, 0, 0, 0, 0, 0, 0, 0];
% for i = 3001 : N
%     for j = 1 : BSN
%         range(i, j) = normrnd(0, sigma(j)) + NLOSB(j) * NLOS(j) + sqrt(dot(BSP(j, :) - r(i, :), BSP(j, :) - r(i, :)));
%     end
% end