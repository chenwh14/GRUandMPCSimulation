%%
load('mex.mat');
mex(lpath,lib,'MPCwrapper.cpp','MPController.cpp','KalmanFilter.cpp')
%%
clipboard('copy',ss16.A);

%%
clipboard('copy',ss16.B);
%%
clipboard('copy',ss16.C);
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
hold on;
histogram(abs(errorX),edgesX,'FaceColor','r','Normalization','cdf');
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

%%
subplot(1,3,1);
errorX=out.errorX*1e6;
meanErrorX=mean(abs(errorX));
rmsErrorX=sqrt(mean(errorX.^2));
maxErrorX=max(abs(errorX));
edgesX=0:50:ceil(maxErrorX/50)*50;
histogram(abs(errorX),edgesX,'Normalization','cdf');

hold on
errorX2=outWithGRU.errorX*1e6;
meanErrorX2=mean(abs(errorX2));
rmsErrorX2=sqrt(mean(errorX2.^2));
maxErrorX2=max(abs(errorX2));
edgesX2=0:50:ceil(maxErrorX2/50)*50;
histogram(abs(errorX2),edgesX2,'Normalization','cdf','FaceColor','r');
axis([0 1.1*max(max(edgesX),max(edgesX2)) 0 1]);
title('x方向跟踪误差分布');

subplot(1,3,2);
errorY=out.errorY*1e6;
meanErrorY=mean(abs(errorY));
rmsErrorY=sqrt(mean(errorY.^2));
maxErrorY=max(abs(errorY));
edgesY=0:50:ceil(maxErrorY/50)*50;
histogram(abs(errorY),edgesY,'Normalization','cdf');
hold on
errorY2=outWithGRU.errorY*1e6;
meanErrorY2=mean(abs(errorY2));
rmsErrorY2=sqrt(mean(errorY2.^2));
maxErrorY2=max(abs(errorY2));
edgesY2=0:50:ceil(maxErrorY2/50)*50;
histogram(abs(errorY2),edgesY2,'Normalization','cdf','FaceColor','r');
axis([0 1.1*max(max(edgesY),max(edgesY2)) 0 1]);
title('y方向跟踪误差分布');

subplot(1,3,3);
error=sqrt(errorX.^2+errorY.^2);
meanError=mean(abs(error));
rmsError=sqrt(mean(error.^2));
sizeOfError=size(error,1);
maxError=max(abs(error));
edges=0:50:ceil(maxError/50)*50;
histogram(abs(error),edges,'Normalization','cdf');

hold on
error2=sqrt(errorX2.^2+errorY2.^2);
meanError2=mean(abs(error2));
rmsError2=sqrt(mean(error2.^2));
maxError2=max(abs(error2));
edges2=0:50:ceil(maxError2/50)*50;
histogram(abs(error2),edges2,'Normalization','cdf','FaceColor','r');

axis([0 1.1*max(max(edges),max(edges2)) 0 1]);
title('总跟踪误差分布');