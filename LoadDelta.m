%%�趨��ƫ��
%�������Ϊ:��ǰʱ��T
%�������Ϊ����ǰʱ�̵���̬��ָ��������̬��(alpha,beta,mu)��
function [DeltaA, DeltaE] = LoadDelta(Time, Signal_Type) %���еĽǶȵ�λΪrad
global h_step;   %%���沽��
global buf;
if Signal_Type ==1  %%��ͬ��ֵ��Ƶ�ʵ������ź�
    Dlt_alp = (20 -0.4*Time)* sin(8* pi* Time/(10 + Time));   %%����ʱ��0-60S
elseif Signal_Type==2  %%��ͬ��ֵ��Ƶ��de ��������ź�
    Dlt_alp = (5 + 0.25*Time)* sin(4* pi* Time/(10 + Time));   %%����ʱ��60s
elseif Signal_Type==3  %%��ͬ��ֵ��Ƶ�ʵ���Ϸ����ź�
    Dlt_alp = (15 - 0.4*Time)* sign(sin(6* pi* Time/(10 + Time)));
elseif Signal_Type==4        
    if Time<=30
         Dlt_alp = 10* sin(pi* Time/10); %%����ʱ��60s
    else 
        Dlt_alp = 10* sign(sin(pi* Time/10));   %%����ʱ��60s
    end
elseif Signal_Type==5
    if Time==0
        load('Dlt_tst.mat','DltAlp');  %%��ôдÿ�ζ�������ļ���Ч�ʺܵ�
        buf = DltAlp;
    end
    Dlt_alp = buf(floor(Time/h_step)+1,1);   
end

DeltaE = [Dlt_alp,0,0];  %%��Ч��

%%��Ч��ת��Ϊʵ�ʶ�
V2R = -1 * [1,0,-1; 1,0,1; 0,1,0];  %%�ɵ�Ч�Ķ�ָ��ת��Ϊʵ�ʵĶ�ƫ ��-1����Ϊ����Ч��ƫ�Ķ���Ϊ���Ķ�ƫ����������̬��
Delta_1 = V2R * DeltaE'; %%��ƫ˳�� e,a,r
DeltaA = Delta_1';
   
end

