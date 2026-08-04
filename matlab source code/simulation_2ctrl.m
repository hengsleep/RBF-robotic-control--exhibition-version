function [sys,x0,str,ts] = spacemodel(t,x,u,flag)

switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes; % 初始化系统尺寸与参数
case 1
    sys=mdlDerivatives(t,x,u);         % 计算连续状态导数（自适应权值更新）
case 3
    sys=mdlOutputs(t,x,u);             % 计算控制总扭矩与不确定性估计值
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
global c b kv kp
sizes = simsizes;
sizes.NumContStates  = 10; % 10个连续状态：RBF神经网络权值矩阵(5x2)
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;  % 输出量：[关节1扭矩, 关节2扭矩, 关节1实际扰动, 关节1估计扰动, 关节2实际扰动, 关节2估计扰动]
sizes.NumInputs      = 10; % 输入量：[qd1, qd2, dqd1, dqd2, q1, dq1, q2, dq2, ddq1, ddq2]
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = 0.1*ones(1,10);  % 神经网络权值初始值
str = [];
ts  = [0 0];

% RBF神经网络高斯基函数中心点矩阵 c 与 宽度 b
c= [-2 -1 0 1 2;
    -2 -1 0 1 2;
    -2 -1 0 1 2;
    -2 -1 0 1 2];
b=3.0;

% PD控制器增益参数
alfa=3;
kp=[alfa^2 0;
    0 alfa^2];
kv=[2*alfa 0;
    0 2*alfa];

function sys=mdlDerivatives(t,x,u)
global c b kv kp

% 误差状态空间系统矩阵 A 和 B
A=[zeros(2) eye(2);
    -kp -kv];
B=[0 0;0 0;1 0;0 1];

% 解 Lyapunov 方程 A'P + PA = -Q，用于构造自适应律
Q=[50 0 0 0; 
   0 50 0 0;
   0 0 50 0;
   0 0  0 50];
P=lyap(A',Q);
eig(P);

% 提取理想轨迹与实际状态
qd1=u(1);
qd2=u(2);
d_qd1=u(3);
d_qd2=u(4);

q1=u(5);dq1=u(6);q2=u(7);dq2=u(8);

% 计算轨迹跟踪的位置误差与速度误差
e1=q1-qd1;
e2=q2-qd2;
de1=dq1-d_qd1;
de2=dq2-d_qd2;

% 重构神经网络权值矩阵 w 与 误差向量 xi
w=[x(1) x(2) x(3) x(4) x(5);x(6) x(7) x(8) x(9) x(10)]';
xi=[e1;e2;de1;de2];

% 计算 RBF 神经元高斯激活函数值 h
h=zeros(5,1);
for j=1:1:5
    h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
gama=20; % 自适应学习率
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S1=2; % 自适应律模式：1-标准Lyapunov自适应律；2-带UUB项的鲁棒自适应律
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if S1==1       % Adaptive Law
    dw=gama*h*xi'*P*B;
elseif S1==2   % Adaptive Law with UUB
    k1=0.001;
    dw=gama*h*xi'*P*B+k1*gama*norm(x)*w;
end
dw=dw';

% 给连续状态导数赋值（权值变化率）
for i=1:1:5
    sys(i)=dw(1,i);
    sys(i+5)=dw(2,i);    
end

function sys=mdlOutputs(t,x,u)
global c b kv kp

% 提取理想轨迹量与期望加速度
qd1=u(1);
qd2=u(2);
d_qd1=u(3);
d_qd2=u(4);

dd_qd1=-0.2*(0.5*pi)^2*sin(0.5*pi*t);
dd_qd2=0.2*(0.5*pi)^2*cos(0.5*pi*t);
dd_qd=[dd_qd1;dd_qd2];

% 提取实际运动状态与实际加速度
q1=u(5);dq1=u(6);q2=u(7);dq2=u(8);

ddq1=u(9);ddq2=u(10);
ddq=[ddq1;ddq2];

e1=q1-qd1;
e2=q2-qd2;
de1=dq1-d_qd1;
de2=dq2-d_qd2;
e=[e1;e2];
de=[de1;de2];

% 机械臂标称动力学模型参数计算 (M0, C0, G0)
v=13.33;
q01=8.98;
q02=8.75;
g=9.8;

M0=[v+q01+2*q02*cos(q2) q01+q02*cos(q2);
    q01+q02*cos(q2) q01];
C0=[-q02*dq2*sin(q2) -q02*(dq1+dq2)*sin(q2);
     q02*dq1*sin(q2)  0];
G0=[15*g*cos(q1)+8.75*g*cos(q1+q2);
    8.75*g*cos(q1+q2)];

dq=[dq1;dq2];

% 基础计算扭矩控制律 tol1
tol1=M0*(dd_qd-kv*de-kp*e)+C0*dq+G0;

% 计算系统的实际模型不确定性与干扰 f
d_M=0.2*M0;
d_C=0.2*C0;
d_G=0.2*G0;
d1=2;d2=3;d3=6;
d=[d1+d2*norm([e1,e2])+d3*norm([de1,de2])];
%d=[20*sin(2*t);20*sin(2*t)];
f=inv(M0)*(d_M*ddq+d_C*dq+d_G+d);

% RBF 神经网络拟合项 fn 计算
xi=[e1;e2;de1;de2];
h=zeros(5,1);
for j=1:1:5
    h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S=3; % 控制方案选择：1-标称控制；2-传统计算扭矩；3-RBF自适应补偿控制
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if S==1               %Nominal model based controller
    tol=tol1;        
elseif S==2           %Modified computed torque controller
    tol2=-M0*f;
    tol=tol1+tol2;
elseif S==3           %RBF compensated controller
    w=[x(1) x(2) x(3) x(4) x(5);x(6) x(7) x(8) x(9) x(10)]';
    fn=w'*h;
    tol2=-M0*fn;
    tol=tol1+1*tol2;
end

% 输出端信号分配
sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=f(1);
sys(4)=fn(1);
sys(5)=f(2);
sys(6)=fn(2);