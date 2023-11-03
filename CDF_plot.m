function [] = CDF_plot(r_BeidouSolve,r_SHKF,r_RWGH_KF,r)
N2=length(r_BeidouSolve);
for i=1:N2
    r_tdoa(i)=sqrt((r_RWGH_KF(i,1) - r(i,1)).^2+(r_RWGH_KF(i,2) - r(i,2)).^2+(r_RWGH_KF(i,3) - r(i,3)).^2);
    r_tdoa2(i) = sqrt((r_SHKF(i,1) - r(i,1)).^2+(r_SHKF(i,2) - r(i,2)).^2+(r_SHKF(i,3) - r(i,3)).^2);
    r_tdoa3(i) = sqrt((r_BeidouSolve(i,1) - r(i,1)).^2+(r_BeidouSolve(i,2) - r(i,2)).^2+(r_BeidouSolve(i,3) - r(i,3)).^2);
end
cdfplot(r_tdoa);
hold on;
cdfplot(r_tdoa2);
hold on;
cdfplot(r_tdoa3);
legend('紧耦合','松耦合','北斗单源');
legend('Location', 'northeast');
xlim([0,20]);



end

