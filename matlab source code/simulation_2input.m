function [sys,x0,str,ts] = spacemodel(t,x,u,flag)

switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes; % 初始化
case 1
    sys=mdlDerivatives(t,x,u);
case 3
    sys=mdlOutputs(t,x,u);             % 输出计算
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4; % 输出：[qd1, qd2, dqd1, dqd2]
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys=mdlOutputs(t,x,u)
% 生成关节1与关节2的期望位置 qd 及期望速度 dqd
qd1=1+0.2*sin(0.5*pi*t);
qd2=1-0.2*cos(0.5*pi*t);
dqd1=0.2*0.5*pi*cos(0.5*pi*t);
dqd2=0.2*0.5*pi*sin(0.5*pi*t);

sys(1)=qd1;
sys(2)=qd2;
sys(3)=dqd1;
sys(4)=dqd2;