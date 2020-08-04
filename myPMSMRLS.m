function [sys,x0,str,ts] = myPMSMRLS(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
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

sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 4;  %输出变量个数
sizes.NumInputs      = 4;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

global IqOld;
IqOld = 0;

global P;  %P矩阵初始化
alpha = 1;
P = alpha*diag([1, 1, 1], 0);

global Delta;
Delta = [0; 0; 1];

function sys=mdlOutputs(t,x,u)   %计算输出子函数

global Delta;
global IqOld;
global P;

lamda = 0.999;
% 仿真时间间隔
Ts = 1e-04;  % 0.0001s

% 输出信号：dq轴电流、电压，转子电角度
Id = u(1);
Iq = u(2);
OmegaE = u(3);
Uq = u(4);

% 构造Iq的微分量
dIq = (Iq - IqOld)/Ts;

% 构造最小二乘法参数矩阵y
y = dIq + OmegaE*Id;

% 构造输入矩阵phi
phi = [-Iq; -OmegaE; Uq];

% 记录上一时刻的Id、Iq，用于求微分
IqOld = Iq;

% 新的P矩阵
P = 1/lamda*(P - (P*phi*(phi')*P)*inv(lamda + (phi')*P*phi));

% 递归迭代估计值
Delta = Delta + P*phi*(y - (phi')*Delta);

a = Delta(1);
b = Delta(2);
c = Delta(3);
R = a/c;
Ke = b/c;
L = 1/c;

sys(1) = R;
sys(2) = Ke;
sys(3) = L;
sys(4) = P(1, 1);