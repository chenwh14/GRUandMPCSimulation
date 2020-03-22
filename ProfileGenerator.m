function [sys,x0,str,ts,simStateCompliance] = ProfileGenerator(t,x,u,flag,ts_,vmax,amax)
switch flag, 
    
    %%%%%%%%%%%%%%%%%%
    % Initialization %
    %%%%%%%%%%%%%%%%%%
    case 0,
        [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(ts_,amax,vmax);
        
        %%%%%%%%%%
        % Update %
        %%%%%%%%%%
    case 2,
        [sys]= mdlUpdate(t,x,u,ts_,vmax,amax);
        
        %%%%%%%%%%
        % Output %
        %%%%%%%%%%
    case 3,
        sys = mdlOutputs(t,x,u);
        
        %%%%%%%%%%%%%
        % Terminate %
        %%%%%%%%%%%%%
    case 9,
        sys = [];
        
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%end sfundsc2

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(ts_,amax,vmax)

n=round(0.1/vmax/ts_);

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 2+2*n;%current position target(1);current velocity target(1);profileBuffer(n);velocityBuffer(n)
sizes.NumOutputs     = 2;%target in this period
sizes.NumInputs      = 4;%u(1):input valid;u(2):relative position command;u(3):current position;u(4):current velocity
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0  = 0;
str = [];
ts  = [ts_ 0];

% speicfy that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

% end mdlInitializeSizes

%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function [sys]= mdlUpdate(t,x,u,Ts,vmax,amax)
n=(size(x,1)-2)/2;
profileBufferLast=x(3:n+2);
velocityBufferLast=x(n+3:2*n+2);
if u(1)==0%no new command input
    %update buffer
    profileBufferNew=[profileBufferLast(2:end);profileBufferLast(end)];
    velocityBufferNew=[velocityBufferLast(2:end);0];
    x(1)=u(3);
    x(2)=u(4);
else if u(1)==1%command received
        s=u(2);
        v=x(2);       
        bufferSize=size(profileBufferLast,1);
        [path,vel]=TrapezoidalPathGen(s,v,Ts,vmax,amax,bufferSize);
        path=path+u(3);
        pathSize=size(path,1);
        if pathSize>0
            profileBufferNew=[path;path(end)*ones(bufferSize-pathSize,1)];
            velocityBufferNew=[vel;zeros(bufferSize-pathSize,1)];
        else
            profileBufferNew=[profileBufferLast(2:end);profileBufferLast(end)];
            velocityBufferNew=[velocityBufferLast(2:end);0];
        end
        x(1)=u(3);
        x(2)=u(4);
    end
end
x(3:n+2)=profileBufferNew;
x(n+3:2*n+2)=velocityBufferNew;
sys = x;

%end mdlUpdate

%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function sys = mdlOutputs(t,x,u)
n=(size(x,1)-2)/2;
y(1)=x(3);
y(2)=x(n+3);
sys = y;

%end mdlOutputs

