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
function [sys,x0,str,ts]=mdlInitializeSizes   %ϵͳ�ĳ�ʼ��
sizes = simsizes;
sizes.NumContStates  = 3;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 3;   %����ϵͳ����ı���
sizes.NumInputs      = 3;   %����ϵͳ����ı���
sizes.DirFeedthrough = 0;   %����ϵͳֱ��ͨ������������һ��Ϊ1
sizes.NumSampleTimes = 0;   % at least one sample time is needed
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [0 0 0 ];            % ϵͳ��ʼ״̬
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��



function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
Ud = u(1);
Uq = u(2);
TL = u(3);

id = x(1);
iq = x(2);
dtheta = x(3);
%ϵͳ�����Ķ���
R = 5.6;
Ld = 0.01157;% H
Lq = 0.01157;% H
L = Ld;
Flux_linkage = 0.125; %wb
Pn = 4;  %������
J1 = 0.384e-4; %kg*m^2
Tn = 1.47; %N*m
N = 4000; %r/min
Je = 1.25e-6; %kg*m^2
J = (J1+Je)/Pn;
B1 = 0;  %kg*m^2/s
Be = 0.001;
B = (B1+Be)/Pn;
M = 1*0.001; %����
xite = L-M;

T =[-R/xite 0 0;
    0 -R/xite (-sqrt(6)/2*Flux_linkage)/xite;
    0 sqrt(6)/2*Flux_linkage/J -B/J];  %3*3

amp = [Ud/xite; Uq/xite; -TL/J];  %3*1
th = [id; iq; dtheta];  %4*1  iu iv iw dtheta

%�˶�����
dth=T*th+amp;

sys(1)=dth(1);   
sys(2)=dth(2);   
sys(3)=dth(3);   
    
function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
sys(1)=x(1);   %id
sys(2)=x(2);   %iq
sys(3)=x(3);   %dtheta

