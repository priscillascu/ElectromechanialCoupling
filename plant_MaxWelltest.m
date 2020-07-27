function [sys,x0,str,ts] = plant_MaxWelltest(t,x,u,flag)

switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case {2,4,9},
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 3;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 3;   %设置系统输出的变量
sizes.NumInputs      = 3;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %设置系统直接通过量的数量，一般为1
sizes.NumSampleTimes = 0;   % at least one sample time is needed
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [0 0 0 ];            % 系统初始状态
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间



function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
Ud = u(1);
Uq = u(2);
TL = u(3);

id = x(1);
iq = x(2);
dtheta = x(3);
%系统参数的定义
R = 5.6;
Ld = 0.01157;% H
Lq = 0.01157;% H
L = Ld;
Flux_linkage = 0.125; %wb
Pn = 4;  %极对数
J1 = 0.384e-4; %kg*m^2
Tn = 1.47; %N*m
N = 4000; %r/min
Je = 1.25e-6; %kg*m^2
J = (J1+Je)/Pn;
B1 = 0;  %kg*m^2/s
Be = 0.001;
B = (B1+Be)/Pn;
M = 1*0.001; %互感
xite = L-M;

T =[-R/xite 0 0;
    0 -R/xite (-sqrt(6)/2*Flux_linkage)/xite;
    0 sqrt(6)/2*Flux_linkage/J -B/J];  %3*3

amp = [Ud/xite; Uq/xite; -TL/J];  %3*1
th = [id; iq; dtheta];  %4*1  iu iv iw dtheta

%运动方程
dth=T*th+amp;

sys(1)=dth(1);   
sys(2)=dth(2);   
sys(3)=dth(3);   
    
function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
sys(1)=x(1);   %id
sys(2)=x(2);   %iq
sys(3)=x(3);   %dtheta

