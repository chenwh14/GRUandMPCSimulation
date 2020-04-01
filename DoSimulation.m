folderNames=['20191226_193003';'20191226_201154';'20191229_162826';'20191231_150128';'20191231_151217';'20191231_155405';'20200103_171355';'20200103_172443';'20200109_195406';'20200109_204357';'20200114_215412';'20200114_221920';'20200115_153335';'20200115_155030'];
pathName='E:\处理过的实验数据\';
for i=1:size(folderNames,1)
    [pathX,pathY]=ConnectClips([pathName,folderNames(i,:)]);
    timeSpan=floor(size(pathX,1)/300);
    %timeSpan=1;
    %%
    simulationNames={'DoubleStageWithGRU'; 'DoubleStageNoPred'; 'DoubleStageRealPred'};
    for j=1:3
        mdl = simulationNames{j};
        load_system(mdl);
        cs = getActiveConfigSet(mdl);
        mdl_cs = cs.copy;
        set_param(mdl_cs,'StartTime','0','StopTime',num2str(timeSpan));
        simout{j}=sim(mdl,mdl_cs);    
        close_system(mdl);
    end
    %%
    save([pathName,folderNames(i,:),'.mat'],'simout');
    %%
    clf;
    subplot(1,3,1);
    errorX=simout{3}.errorX*1e6;%采用真实值作为MPC输入时的误差
    meanErrorX=mean(abs(errorX));
    rmsErrorX=sqrt(mean(errorX.^2));
    maxErrorX=max(abs(errorX));
    edgesX=0:50:ceil(maxErrorX/50)*50;
    histogram(abs(errorX),edgesX,'Normalization','cdf','FaceColor','w');
    
    hold on
    errorX2=simout{1}.errorX*1e6;%采用GRU预测值作为MPC输入时的误差
    meanErrorX2=mean(abs(errorX2));
    rmsErrorX2=sqrt(mean(errorX2.^2));
    maxErrorX2=max(abs(errorX2));
    edgesX2=0:50:ceil(maxErrorX2/50)*50;
    histogram(abs(errorX2),edgesX2,'Normalization','cdf','FaceColor','#0072BD');
    
    hold on
    errorX3=simout{2}.errorX*1e6;%不使用预测时的误差
    meanErrorX3=mean(abs(errorX3));
    rmsErrorX3=sqrt(mean(errorX3.^2));
    maxErrorX3=max(abs(errorX3));
    edgesX3=0:50:ceil(maxErrorX3/50)*50;
    histogram(abs(errorX3),edgesX3,'Normalization','cdf','FaceColor','#D95319');
    
    axis([0 1.1*max([max(edgesX),max(edgesX2),max(edgesX3)]) 0 1]);
    title('x方向跟踪误差分布');
    
    subplot(1,3,2);
    errorY=simout{3}.errorY*1e6;
    meanErrorY=mean(abs(errorY));
    rmsErrorY=sqrt(mean(errorY.^2));
    maxErrorY=max(abs(errorY));
    edgesY=0:50:ceil(maxErrorY/50)*50;
    histogram(abs(errorY),edgesY,'Normalization','cdf','FaceColor','w');
    hold on
    errorY2=simout{1}.errorY*1e6;
    meanErrorY2=mean(abs(errorY2));
    rmsErrorY2=sqrt(mean(errorY2.^2));
    maxErrorY2=max(abs(errorY2));
    edgesY2=0:50:ceil(maxErrorY2/50)*50;
    histogram(abs(errorY2),edgesY2,'Normalization','cdf','FaceColor','#0072BD');
    
    hold on
    errorY3=simout{2}.errorY*1e6;%不使用预测时的误差
    meanErrorY3=mean(abs(errorY3));
    rmsErrorY3=sqrt(mean(errorY3.^2));
    maxErrorY3=max(abs(errorY3));
    edgesY3=0:50:ceil(maxErrorY3/50)*50;
    histogram(abs(errorY3),edgesY3,'Normalization','cdf','FaceColor','#D95319');
    axis([0 1.1*max([max(edgesY),max(edgesY2),max(edgesY3)]) 0 1]);
    title('y方向跟踪误差分布');
    
    subplot(1,3,3);
    error=sqrt(errorX.^2+errorY.^2);
    meanError=mean(abs(error));
    rmsError=sqrt(mean(error.^2));
    sizeOfError=size(error,1);
    maxError=max(abs(error));
    edges=0:50:ceil(maxError/50)*50;
    histogram(abs(error),edges,'Normalization','cdf','FaceColor','w');
    
    hold on
    error2=sqrt(errorX2.^2+errorY2.^2);
    meanError2=mean(abs(error2));
    rmsError2=sqrt(mean(error2.^2));
    maxError2=max(abs(error2));
    edges2=0:50:ceil(maxError2/50)*50;
    histogram(abs(error2),edges2,'Normalization','cdf','FaceColor','#0072BD');
    
    hold on
    error3=sqrt(errorX3.^2+errorY3.^2);%不使用预测时的误差
    meanError3=mean(abs(error3));
    rmsError3=sqrt(mean(error3.^2));
    maxError3=max(abs(error3));
    edges3=0:50:ceil(maxError3/50)*50;
    histogram(abs(error3),edges3,'Normalization','cdf','FaceColor','#D95319');
    
    axis([0 1.1*max([max(edges),max(edges2),max(edges3)]) 0 1]);
    title('总跟踪误差分布');
    set(gcf,'unit','centimeters','position',[3 5 40 10]);
    print(gcf,[pathName,folderNames(i,:),'_MPCCorrected'],'-dpng','-r400');
end