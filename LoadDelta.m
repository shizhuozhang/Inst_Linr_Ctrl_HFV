%%设定舵偏角
%输入参数为:当前时刻T
%输出参数为：当前时刻的姿态角指令气动姿态角(alpha,beta,mu)，
function [DeltaA, DeltaE] = LoadDelta(Time, Signal_Type) %其中的角度单位为rad
global h_step;   %%仿真步长
global buf;
if Signal_Type ==1  %%不同幅值和频率的正弦信号
    Dlt_alp = (20 -0.4*Time)* sin(8* pi* Time/(10 + Time));   %%仿真时长0-60S
elseif Signal_Type==2  %%不同幅值和频率de 组合正弦信号
    Dlt_alp = (5 + 0.25*Time)* sin(4* pi* Time/(10 + Time));   %%仿真时长60s
elseif Signal_Type==3  %%不同幅值和频率的组合方波信号
    Dlt_alp = (15 - 0.4*Time)* sign(sin(6* pi* Time/(10 + Time)));
elseif Signal_Type==4        
    if Time<=30
         Dlt_alp = 10* sin(pi* Time/10); %%仿真时长60s
    else 
        Dlt_alp = 10* sign(sin(pi* Time/10));   %%仿真时长60s
    end
elseif Signal_Type==5
    if Time==0
        load('Dlt_tst.mat','DltAlp');  %%这么写每次都会加载文件，效率很低
        buf = DltAlp;
    end
    Dlt_alp = buf(floor(Time/h_step)+1,1);   
end

DeltaE = [Dlt_alp,0,0];  %%等效舵

%%等效舵转化为实际舵
V2R = -1 * [1,0,-1; 1,0,1; 0,1,0];  %%由等效的舵指令转化为实际的舵偏 乘-1是因为，等效舵偏的定义为正的舵偏产生正的姿态角
Delta_1 = V2R * DeltaE'; %%舵偏顺序 e,a,r
DeltaA = Delta_1';
   
end

