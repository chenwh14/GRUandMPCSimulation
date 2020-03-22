function [sys,x0,str,ts,simStateCompliance] = CommandGenerator(t,x,u,flag,ts_,command_period)
%SFUNDSC2 Example unit delay MATLAB File S-function
%   The MATLAB file S-function is an example of how to implement a unit
%   delay.
%
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.

%   Copyright 1990-2009 The MathWorks, Inc.
switch flag,
    
    %%%%%%%%%%%%%%%%%%
    % Initialization %
    %%%%%%%%%%%%%%%%%%
    case 0,
        [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(ts_);
        
        %%%%%%%%%%
        % Update %
        %%%%%%%%%%
    case 2,
        sys= mdlUpdate(t,x,u);
        
        %%%%%%%%%%
        % Output %
        %%%%%%%%%%
    case 3,
        sys = mdlOutputs(t,x,u,ts_,command_period);
        
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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(ts_)

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0  = [];
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
function sys= mdlUpdate(t,x,u)
sys = [];

%end mdlUpdate

%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function sys = mdlOutputs(t,x,u,ts_,command_period)
while t>=command_period
    t=t-command_period;
end
if t<ts_
    sys = [1;u(1)];
else
    sys=[0;0];
end

%end mdlOutputs

