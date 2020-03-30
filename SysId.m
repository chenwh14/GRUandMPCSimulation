%%
clipboard('copy',ss2.A);

%%
clipboard('copy',ss2.B);
%%
clipboard('copy',ss2.C);
%%
error=out.error*1e6;
meanError=mean(abs(error));
rmsError=sqrt(mean(error.^2));
maxError=max(abs(error));
edges=0:50:ceil(maxError/50)*50;
histogram(abs(error),edges,'Normalization','cdf');
%%
subplot(1,3,1);
errorX=out.errorX;
errorX=errorX(300:end);%前1秒忽略
meanErrorX=mean(abs(errorX));
rmsErrorX=sqrt(mean(errorX.^2));
maxErrorX=max(abs(errorX));
edgesX=0:50e-6:ceil(maxErrorX/50e-6)*50e-6;
histogram(abs(errorX),edgesX,'Normalization','cdf');
axis([0 1.1*max(edgesX) 0 1]);
title('x方向跟踪误差分布');

subplot(1,3,2);
errorY=out.errorY;
errorY=errorY(300:end);%前1秒忽略
meanErrorY=mean(abs(errorY));
rmsErrorY=sqrt(mean(errorY.^2));
maxErrorY=max(abs(errorY));
edgesY=0:50e-6:ceil(maxErrorY/50e-6)*50e-6;
histogram(abs(errorY),edgesY,'Normalization','cdf');
axis([0 1.1*max(edgesY) 0 1]);
title('y方向跟踪误差分布');

subplot(1,3,3);
error=sqrt(errorX.^2+errorY.^2);
meanError=mean(abs(error));
rmsError=sqrt(mean(error.^2));
sizeOfError=size(error,1);
maxError=max(abs(error));
edges=0:50e-6:ceil(maxError/50e-6)*50e-6;
histogram(abs(error),edges,'Normalization','cdf');
axis([0 1.1*max(edges) 0 1]);
title('总跟踪误差分布');

