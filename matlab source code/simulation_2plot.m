close all; % 关闭所有绘制窗口

% 图1：关节1与关节2的位置跟踪性能对比
figure(1);
subplot(211);
plot(t,x(:,1),'r',t,x(:,5),'k:','linewidth',2);
xlabel('time(s)');ylabel('Position tracking for link 1');
legend('ideal position for link 1','position tracking for link 1');
subplot(212);
plot(t,x(:,2),'r',t,x(:,7),'k:','linewidth',2);
xlabel('time(s)');ylabel('Position tracking for link 2');
legend('ideal position for link 2','position tracking for link 2');

% 图2：关节1与关节2的速度跟踪性能对比
figure(2);
subplot(211);
plot(t,x(:,3),'r',t,x(:,6),'k:','linewidth',2);
xlabel('time(s)');ylabel('Speed tracking for link 1');
legend('ideal speed for link 1','speed tracking for link 1');
subplot(212);
plot(t,x(:,4),'r',t,x(:,8),'k:','linewidth',2);
xlabel('time(s)');ylabel('Speed tracking for link 2');
legend('ideal speed for link 2','speed tracking for link 2');

% 图3：控制器的总输出扭矩
figure(3);
subplot(211);
plot(t,tol(:,1),'r','linewidth',2);
xlabel('time(s)');ylabel('Control input of link 1');
subplot(212);
plot(t,tol(:,1),'r','linewidth',2);
xlabel('time(s)');ylabel('Control input of link 2');

% 图4：RBF神经网络对系统不确定性 f 的估计与逼近效果对比
figure(4);
subplot(211);
plot(t,f(:,1),'r',t,f(:,2),'k:','linewidth',2);
xlabel('time(s)');ylabel('f1 and fn1');
legend('Practical uncertainties of link 1','Estimation uncertainties of link 1');
subplot(212);
plot(t,f(:,3),'r',t,f(:,4),'k:','linewidth',2);
xlabel('time(s)');ylabel('f2 and fn2');
legend('Practical uncertainties of link 2','Estimation uncertainties of link 2');