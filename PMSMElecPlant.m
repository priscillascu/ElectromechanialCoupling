% 永磁同步电机机电耦合仿真模型

function [sys,x0,str,ts] = PMSMElecPlant(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=mdlDerivatives(t,x,u);   %调用计算微分子函数
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数

sizes = simsizes;

sizes.NumContStates  = 8;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 4;  %输出变量个数
sizes.NumInputs      = 4;   %输入变量个数
sizes.DirFeedthrough = 0;   %输入信号是否在输出端出现
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [0, 0, 0, 0, 0, 0, 0, 0];   %初始值
str = [];   
ts  = [0 0];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

global theta;
theta = 0;

function sys = mdlDerivatives(t, x, u)    %计算微分子函数
global theta;
ia = x(1);
ib = x(2);
ic = x(3);
dtheta = x(4);

CtrlIn = u(1 : 4);

i = [ia; ib; ic];

L = 11.57;  %定子互感
M = 1;   %定子自感
phi = 0.125;  %转子磁通

Ru = 5.6;
Rv = 5.6;
Rw = 5.6;
R = diag([Ru, Rv, Rw], 0);  %定子电阻

A = [L, M, M;
      M, L, M;
      M, M, L];
  
C = [-phi*sin(theta);
    -phi*sin(theta - 2/3*pi);
    -phi*sin(theta + 2/3*pi)];

D = [-phi*sin(theta), -phi*sin(theta - 2/3*pi), -phi*sin(theta + 2/3*pi)];

J = 0.384e-04 + 1.25e-06;
B = 0 + 0.001;

E = [-inv(A)*R, -inv(A)*C;
            inv(J)*D -inv(J)*B];
        
F = [inv(A), [0; 0; 0];
        0, 0, 0, -1];
    
x1 = [i; dtheta];
dx1 = E*x1 + F*CtrlIn;

theta = mod(theta + dtheta*0.0001, 2*pi);

% sys(1) = dx1(1);
% sys(2) = dx1(2);
% sys(3) = dx1(3);
% sys(4) = dx1(4);
sys(1) = ia;
sys(2) = dx1(1);
sys(3) = ib;
sys(4) = dx1(2);
sys(5) = ic;
sys(6) = dx1(3);
sys(7) = dtheta;
sys(8) = dx1(4);


function sys=mdlOutputs(t,x,u)   %计算输出子函数
sys(1) = x(1);  %位置输出
sys(2) = x(3);  %位置输出
sys(3) = x(5);  %位置输出
sys(4) = x(7);  %位置输出