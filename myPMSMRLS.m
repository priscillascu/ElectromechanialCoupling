function [sys,x0,str,ts] = myPMSMRLS(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % ���ó�ʼ���Ӻ���
  case 1,
    sys=[];
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

sizes.NumContStates  = 0;  %����״̬��������
sizes.NumDiscStates  = 0;  %��ɢ״̬��������
sizes.NumOutputs     = 4;  %�����������
sizes.NumInputs      = 4;   %�����������
sizes.DirFeedthrough = 1;   %�����ź��Ƿ�������Ӻ����г���
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %��ʼֵ
str = [];   
ts  = [];   %[0 0]��������ϵͳ��[-1 0]��ʾ�̳���ǰ�Ĳ���ʱ������
simStateCompliance = 'UnknownSimState';

global IqOld;
IqOld = 0;

global P;  %P�����ʼ��
alpha = 1;
P = alpha*diag([1, 1, 1], 0);

global Delta;
Delta = [0; 0; 1];

function sys=mdlOutputs(t,x,u)   %��������Ӻ���

global Delta;
global IqOld;
global P;

lamda = 0.999;
% ����ʱ����
Ts = 1e-04;  % 0.0001s

% ����źţ�dq���������ѹ��ת�ӵ�Ƕ�
Id = u(1);
Iq = u(2);
OmegaE = u(3);
Uq = u(4);

% ����Iq��΢����
dIq = (Iq - IqOld)/Ts;

% ������С���˷���������y
y = dIq + OmegaE*Id;

% �����������phi
phi = [-Iq; -OmegaE; Uq];

% ��¼��һʱ�̵�Id��Iq��������΢��
IqOld = Iq;

% �µ�P����
P = 1/lamda*(P - (P*phi*(phi')*P)*inv(lamda + (phi')*P*phi));

% �ݹ��������ֵ
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