function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes; % 初始化
case 1
    sys=mdlDerivatives(t,x,u);         % 状态方程状态求导
case 3
    sys=mdlOutputs(t,x,u);             % 状态输出
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 4; % 4个连续状态：[q1, dq1, q2, dq2]
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4; % 输出：[q1, dq1, q2, dq2]
sizes.NumInputs      = 6; % 输入：[控制扭矩tol1, tol2, 参考轨迹参数]
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[0.6;0.3;0.5;0.5]; % 系统初始状态
str=[];
ts=[];

function sys=mdlDerivatives(t,x,u)
persistent ddx1 ddx2 % 保持上一时刻的加速度值
if t==0
    ddx1=0;
    ddx2=0;
end

% 计算实时期望轨迹与跟踪误差
qd1=1+0.2*sin(0.5*pi*t);
dqd1=0.2*0.5*pi*cos(0.5*pi*t);
qd2=1-0.2*cos(0.5*pi*t);
dqd2=0.2*0.5*pi*sin(0.5*pi*t);

e1=x(1)-qd1;
e2=x(3)-qd2;
de1=x(2)-dqd1;
de2=x(4)-dqd2;

% 真实机械臂物理动力学参数矩阵
v=13.33;
q1=8.98;
q2=8.75;
g=9.8;

M0=[v+q1+2*q2*cos(x(3)) q1+q2*cos(x(3));
    q1+q2*cos(x(3)) q1];
C0=[-q2*x(4)*sin(x(3)) -q2*(x(2)+x(4))*sin(x(3));
     q2*x(2)*sin(x(3))  0];
G0=[15*g*cos(x(1))+8.75*g*cos(x(1)+x(3));
   8.75*g*cos(x(1)+x(3))];

% 摄动与外部干扰计算
d_M=0.2*M0;
d_C=0.2*C0;
d_G=0.2*G0;

d1=2;d2=3;d3=6;
d=[d1+d2*norm([e1,e2])+d3*norm([de1,de2])];
%d=20*sin(2*t);

% 获取传入的控制扭矩
tol(1)=u(1);
tol(2)=u(2);

dq=[x(2);x(4)];
ddq=[ddx1;ddx2];

% 计算总的不确定性项 f
f=inv(M0)*(d_M*ddq+d_C*dq+d_G+d);

% 计算机械臂系统的实际加速度 ddx
ddx=inv(M0)*(tol'-C0*dq-G0)+1*f;

% 状态微分赋值：dq1, ddq1, dq2, ddq2
sys(1)=x(2);
sys(2)=ddx(1);
sys(3)=x(4);
sys(4)=ddx(2);

% 保存当前加速度，供下一时刻微分计算使用
ddx1=ddx(1);
ddx2=ddx(2);

function sys=mdlOutputs(t,x,u)
% 输出当前机械臂的状态向量
sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);