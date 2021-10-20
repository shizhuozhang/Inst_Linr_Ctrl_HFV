%%设定制导指令
%输入参数为:当前时刻T
%输出参数为：当前时刻的姿态角指令气动姿态角(alpha,beta,mu)，
function ComdAng = LoadCommand(Time) %其中的角度单位为rad

global ctrl_case; %%选择仿真情形，分为六自由度仿真和三自由度仿真
global Signal_Type;

% global sim_case; %%选择仿真情形，分为六自由度仿真和三自由度仿真
global Rad2Deg;  %%弧度转换为度
global ComdAng_lst; %记录上一次的角度
global InAng_lst; %记录上一次的输入角度
Com_beta = 0;    %侧滑始终设定为0
Filter_flag =0;
if Time==0
    if ctrl_case ==1  
        ComdAng_lst = [0,0]/Rad2Deg; %初始化参数
        InAng_lst = [0,0]/Rad2Deg; %初始化参数
    else
         ComdAng_lst = [0,0]; %初始化参数   
         InAng_lst = [0,0]; %初始化参数
    end
end

%% 100s全弹道仿真 自己按感觉设计的制导指令
if ctrl_case ==1  
    if (Time >= 0)&&(Time <= 1)   
        In_alpha = 0/Rad2Deg; %攻角指令
        In_mu = 0/Rad2Deg;    %倾侧角指令
    elseif (Time > 1)&&(Time <= 10) 
        In_alpha = (2*(Time-1))/Rad2Deg; %攻角指令 斜率为-2的斜坡指令
        In_mu = 0;    %倾侧角指令
    elseif (Time > 10)&&(Time <= 25) 
        In_alpha = 18/Rad2Deg; %攻角指令
        In_mu = 0;    %倾侧角指令        
    elseif (Time > 25)&&(Time <= 35) 
        In_alpha = (-Time + 43)/Rad2Deg; %攻角指令 斜率为1的斜坡指令
        In_mu = 0;    %倾侧角指令        
    elseif (Time > 35)&&(Time <= 50) 
        In_alpha = 8/Rad2Deg; %攻角指令
        In_mu = 0;    %倾侧角指令     
    elseif (Time > 50)&&(Time <= 64) 
        In_alpha = 8/Rad2Deg; %攻角指令
        In_mu = (-Time + 50)/Rad2Deg;    %倾侧角指令        
     elseif (Time > 64)&&(Time <= 80) 
        In_alpha = 8/Rad2Deg; %攻角指令
        In_mu = -14/Rad2Deg; %倾侧角指令 斜率为1.2的斜坡指令            
    elseif (Time > 80)&&(Time <= 90) 
        In_alpha = 8/Rad2Deg; %攻角指令
        In_mu = (2*Time -174)/Rad2Deg; %倾侧角指令 斜率为1.2的斜坡指令
    elseif (Time > 90)&&(Time <= 100) 
        In_alpha = 8/Rad2Deg; %攻角指令
        In_mu = 6/Rad2Deg; %倾侧角指令 斜率为1.2的斜坡指令
    else
        In_alpha = 0; %攻角指令
        In_mu = 0;    %倾侧角指令       
    end

  ComdAng(1) = In_alpha; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;    
%% 单点仿真   单纯的阶跃指令 加了一个一阶惯性滤波环节
else
    In_mu = 0;    %倾侧角指令
    if Signal_Type==11  %%幅值变化的正弦信号
        In_alpha = (2 + 0.3*Time)* sin(4* pi* Time/10)/Rad2Deg;   %%频率1
    elseif Signal_Type==1  %%幅值变化的正弦信号
        In_alpha = (2 + 0.3*Time)* sin(3* pi* Time/10)/Rad2Deg;   %%频率1
    elseif Signal_Type==2 %%幅值变化的正弦信号
        In_alpha = (2 + 0.3*Time)* sin(2* pi* Time/10)/Rad2Deg;   %%频率2
    elseif Signal_Type==3 %%幅值变化的正弦信号
        In_alpha = (2 + 0.3*Time)* sin( pi* Time/10)/Rad2Deg;   %%频率3
    elseif Signal_Type==4  %%幅值变化的正弦信号
        In_alpha = (2 + 0.3*Time)* sin( 0.5*pi* Time/10)/Rad2Deg;   %%频率4
    elseif Signal_Type==5 %%幅值变化的方波信号
        In_alpha = (2 + 0.15*Time)* sign(sin( 3*pi* Time/10))/Rad2Deg;   %%频率1
        Filter_flag =1;
    elseif Signal_Type==6  %%幅值变化的方波信号
        In_alpha = (2 + 0.15*Time)* sign(sin( 2*pi* Time/10))/Rad2Deg;   %%频率2
        Filter_flag =1;
     elseif Signal_Type==7 %%幅值变化的方波信号
        In_alpha = (2 + 0.15*Time)* sign(sin( 1*pi* Time/10))/Rad2Deg;   %%频率3
        Filter_flag =1;
    elseif Signal_Type==8 %%幅值变化的方波信号
        In_alpha = (2 + 0.15*Time)* sign(sin( 0.5*pi* Time/10))/Rad2Deg;   %%频率4
        Filter_flag =1;
    elseif Signal_Type==9 
        if (Time >= 0)&&(Time <= 20)   
            In_alpha = 10* sin(1.5* pi* Time/10)/Rad2Deg; %攻角指令
        else
            In_alpha = 5* sign(sin(0.8*pi* Time/10))/Rad2Deg;   %%频率4
        end         
        Filter_flag =1;
    else
        In_alpha = 0; %攻角指令          
    end
       
% % % % % %    指令加一个滤波环节，变成连续可导信号 时间常数0.4s
    if Filter_flag ==1
        Com_alpha_1 = 0.9876*ComdAng_lst(1) + 0.0124* InAng_lst(1); %控制，步长0.05     
        InAng_lst(1) = In_alpha;  
        ComdAng_lst(1) = Com_alpha_1;         
        ComdAng(1) = Com_alpha_1; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;   
    else
        ComdAng(1) = In_alpha; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;    
    end
end
   
end

