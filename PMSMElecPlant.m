% ����ͬ�����������Ϸ���ģ��

function [sys,x0,str,ts] = PMSMElecPlant(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=mdlDerivatives(t,x,u);   %���ü���΢���Ӻ���
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %��������Ӻ���
  case 4,
    sys=[];   %������һ����ʱ���Ӻ���
  case 9,
    sys=[];    %��ֹ�����Ӻ���
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 8;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 4;  %�����������
sizes.NumInputs      = 4;   %�����������
sizes.DirFeedthrough = 0;   %�����ź��Ƿ�������˳���
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [0, 0, 0, 0, 0, 0, 0, 0];   %��ʼֵ
str = [];   
ts  = [0 0];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

global theta;
theta = 0;

function sys = mdlDerivatives(t, x, u)    %����΢���Ӻ���
global theta;
ia = x(1);
ib = x(2);
ic = x(3);
dtheta = x(4);

CtrlIn = u(1 : 4);

i = [ia; ib; ic];

L = 11.57;  %���ӻ���
M = 1;   %�����Ը�
phi = 0.125;  %ת�Ӵ�ͨ

Ru = 5.6;
Rv = 5.6;
Rw = 5.6;
R = diag([Ru, Rv, Rw], 0);  %���ӵ���

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


function sys=mdlOutputs(t,x,u)   %��������Ӻ���
sys(1) = x(1);  %λ�����
sys(2) = x(3);  %λ�����
sys(3) = x(5);  %λ�����
sys(4) = x(7);  %λ�����