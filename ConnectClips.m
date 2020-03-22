%%
%读取文件
clear all;
pathName=uigetdir('请选择文件夹');
fileName=[pathName,'\newXYdataAll.txt'];
[findFly,stageX, stageY, headX, headY, commandX, commandY] = textread(fullfile(fileName), '%d,%f,%f,%f,%f,%f,%f');
%%
freq=300;%伺服控制频率
a11=0.0001777326832096361;a12=0.005308842744342313;a21=0.00534395497810797;a22=-0.00019522174569135787;
centerX=208;
centerY=208;
%%
headErrorX=centerX-headX;headErrorY=centerY-headY;
stageErrorX=headErrorX * a11 + a12 * headErrorY;
stageErrorY=headErrorX * a21 + a22 * headErrorY;
stageTargetX=stageX+stageErrorX;
stageTargetY=stageY+stageErrorY;

%%
%读取可用片段
clipFileName=[pathName,'\可用片段.txt'];
[clipBegin,clipEnd]=textread(fullfile(clipFileName), '%d-%d');
processIndex=1;
diffX=stageTargetX(clipBegin(1));
diffY=stageTargetY(clipBegin(1));
patchLength=round(max(abs(diffX),abs(diffY))/5*freq);
connectedHeadPosition(processIndex:processIndex+patchLength-1,1:2)=[linspace(0,diffX,patchLength)',linspace(0,diffY,patchLength)'];
processIndex=patchLength+processIndex;
connectedHeadPosition(processIndex:clipEnd(1)-clipBegin(1)+processIndex,:)=[stageTargetX(clipBegin(1)+1:clipEnd(1)+1),stageTargetY(clipBegin(1)+1:clipEnd(1)+1)];
processIndex=processIndex+clipEnd(1)-clipBegin(1)+1;
%%
diffX=0;diffY=0;
for i=2:size(clipBegin,1)
    diffX=connectedHeadPosition(processIndex-1,1)-stageTargetX(clipBegin(i)+1);
    diffY=connectedHeadPosition(processIndex-1,2)-stageTargetY(clipBegin(i)+1);
    patchLength=round(max(abs(diffX),abs(diffY))/5*freq);
    connectedHeadPosition(processIndex:processIndex+patchLength-1,1:2)=[linspace(connectedHeadPosition(processIndex-1,1),stageTargetX(clipBegin(i)+1),patchLength)',linspace(connectedHeadPosition(processIndex-1,2),stageTargetY(clipBegin(i)+1),patchLength)'];
    processIndex=patchLength+processIndex;
    connectedHeadPosition(processIndex:processIndex+clipEnd(i)-clipBegin(i),:)=...
        [stageTargetX(clipBegin(i)+1:clipEnd(i)+1),stageTargetY(clipBegin(i)+1:clipEnd(i)+1)];
    processIndex=processIndex+clipEnd(i)-clipBegin(i)+1;
end
%%
% connectedClipFileName=[pathName,'\clip',num2str(i),'.txt'];
% seperatedClipFileID=fopen(seperatedClipFileName,'w');
pathX=[(0:1/300:(size(connectedHeadPosition,1)-1)/300)',connectedHeadPosition(:,1)/1000];
pathY=[(0:1/300:(size(connectedHeadPosition,1)-1)/300)',connectedHeadPosition(:,2)/1000];
