%%校正网络控制器
%双环控制，输入：指令角，实际角，角速度
%输出：角度控制误差，角速度控制误差，角加速度，等效舵偏
% alpha = FltSdt_F(4);   beta = FltSdt_F(5);   mu = FltSdt_F(6); %构成气动角矢量 AeroAng = [alpha, beta, mu];
% q = FltSdt_F(7);   r = FltSdt_F(8);   p = FltSdt_F(9); %构成角速率矢量 Omega = [q ,r, p];
%Cmd_alp = CtrlSdt_B(1);  Cmd_bet = CtrlSdt_B(2);    Cmd_mu = CtrlSdt_B(3); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
%Ero_pit = CtrlSdt_P(1);  Ero_yaw = CtrlSdt_P(2);    Ero_rol = CtrlSdt_P(3); %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
%Ero_qw = CtrlSdt_P(4);  Ero_rw = CtrlSdt_P(5);    Ero_pw = CtrlSdt_P(6); %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
%Acc_pit = CtrlSdt_P(7);  Acc_yaw = CtrlSdt_P(8);    Acc_rol = CtrlSdt_P(9); %构成角加速度矢量 Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
%Dlt_alp = CtrlSdt_P(10);  Dlt_bet = CtrlSdt_P(11);    Dlt_mu = CtrlSdt_P(12); %构成等效舵偏矢量 DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
function [Eror_ang, Eror_omg, DeltaE, CtrlPara] = Ctrller_GMVCPID(CmdAng, AeroAng, Omega, NNpara)

global h_step;   %%仿真步长
global step;     %%记录仿真步数,用于增益调度
global Rad2Deg;  %%弧度转换为度
global UDP_flag;  %%更新参数的标志位

%新定义的全局变量，为了记录中间值而不被清除
global u_lst; %三通道的内环网络上次输入记录值
global Dlt_lst; %三通道的内环网络上次输出记录值
global Kinp_x; %内环控制器参数
global Kiny_x; %内环控制器参数
global Kinr_x; %内环控制器参数

%自适应PID相关的变量
global e_lst2;
global e_lst1;
global K;
global GMVCtemp; 
global K_init;
global Cof_Mu;
global SlidWd_eC;  %%窗口缓存区，用于计算窗口误差积分
global eC_SlidSum;  %%窗口误差积分
global Num_Count1;  %计数
global grd_K;

if step==1
    e_lst1 = 0;
    e_lst2 = 0;
    kp = -60;  tau_i = 2.5;  tau_d = 0.2;  %%这个初始参数还可以  %%60,2.5,0.2%%40,2.5,0.15
    k1 = kp* (1 + h_step/2/tau_i + tau_d/h_step); 
    k2 = - kp* (1 - h_step/2/tau_i + 2*tau_d/h_step);
    k3 = kp* tau_d/ h_step;
    K =[k1, k2, k3]';
    GMVCtemp = 0;
    K_init = K;
    Cof_Mu = 0.5;  %%待调系数
    SlidWd_eC = zeros(1,10);
    eC_SlidSum = 0;
    Num_Count1 = 0;
    grd_K = zeros(3,1);
    
    u_lst = [0,0,0]; %初始化参数
    Dlt_lst = [0,0,0]; %初始化参数
    load Kin_para_N.mat    %内环增益'Kinp','Kiny','Kinr'  插值范围0: 0.01: 100;
    Kinp_x = Kinp;
    Kiny_x = Kiny;
    Kinr_x = Kinr;   
end

% Kinpt = Kinp_x(round((step-1)/2) +1); %获取插值的内环增益
% Koutp = 5; numpz = [0.08343,-0.06910]; denpz = [1,-0.98567];
Kinyt = Kiny_x(round((step-1)/2) +1); %获取插值的内环增益
Kouty = -5; numyz = [0.08343,-0.06910]; denyz = [1,-0.98567];
Kinrt = Kinr_x(round((step-1)/2) +1); %获取插值的内环增益
Koutr = 5;  numrz = [0.03101,-0.03013]; denrz = [1,-0.99912];

%% 计算状态
Eror_ang = CmdAng - AeroAng;  %角度控制误差，期望指令角减实际角度
q_act = Omega(1); r_act = Omega(2); p_act = Omega(3); %实际角速度
%% 俯仰通道控制 角度alpha，角速度q  使用PID控制结构 广义最小方差控制自适应调整最优的PID参数 单环控制
                      
e_now = Eror_ang(1) *Rad2Deg;   %角度控制误差  由于辨识输出用的单位deg，因此这里统一转为deg处理
E = [e_now, e_lst1, e_lst2]';
 du = E'*K_init; %%计算PID控制器输出的增量
% du = E'*K; %%计算PID控制器输出的增量
Dlt_a = Dlt_lst(1) + du;

e_lst2 = e_lst1;  e_lst1 = e_now;
Dlt_lst(1) = Dlt_a; %更新本次记录值

        %%%%计算窗口误差积分，用于判断是否应该变参    
if(rem(step,10)==0)  %%每0.05秒更新一个数据，
    Num_Count1 = Num_Count1 +1;   
    x = rem(Num_Count1,10);  %%%窗口设置为0.5S，一共有10个数据
    if (x==0)
        x = 10;  
    end
    SlidWd_eC(1,x) = abs(e_now);  %%用这种方法滑动窗口，有点取巧
    eC_SlidSum = sum(SlidWd_eC);  %%累加求和
end
           %%%%利用广义最小方差最优控制指标  自适应调整俯仰通道的增益
CgRt = 0.1;

e_idtf = NNpara(1);   a1 = NNpara(2);  a2 = NNpara(3);  b1 = NNpara(4);  b2 = NNpara(5);  bias = NNpara(6); 
uk_lst1 = NNpara(7);  uk_now = NNpara(8);   yk_lst1 = NNpara(9);  yk_now  = NNpara(10);  

%     if abs(e_idtf) > 0.1
%         Cof_Mu = 0.5*Cof_Mu;
%     elseif abs(e_idtf) < 0.005
%         Cof_Mu = 2*Cof_Mu;
%     end

GMVCtemp = CmdAng(1)*Rad2Deg + a1*yk_now + a2*yk_lst1- b1*uk_lst1 - b2*uk_lst1 - bias;

%%不用绝对求解的最优参数，而是采用梯度法求参数的增量
grd_K = (b1*b1 + Cof_Mu).* E*E'*K - GMVCtemp*b1*E;   %%计算梯度

if (eC_SlidSum >= 0.5)%&&(UDP_flag==1)   %%%连续0.5秒的误差都大于0.1度时,才进行参数调整 
    dK = - CgRt .* grd_K;
    K = K + dK;
%    UDP_flag = 0; %%更新完参数，标志位清零
end

%     cof = GMVCtemp*b1/(b1*b1 + Cof_Mu);
%     tmp1 = E*E';    %%这一步由于步长较小，前后几步的误差在数值上相差不大，导致结果矩阵近似奇异矩阵，只能求广义逆
%     if(tmp1(1,1)==0)||(tmp1(2,2)==0)||(tmp1(3,3)==0) 
%         tmp1 = tmp1 + diag(0.000001*ones(3,1)); 
%     end
%     tmp2 = pinv(tmp1);
%     tmp3 = tmp2 *E;
%     K = cof.* tmp3;  %%按照论文中的推导计算最优参数            

kp = 0.5* (K(1) - K(2) - 3*K(3));
tau_i = h_step* kp / (K(1) + K(2) + K(3));
tau_d = h_step* K(3) / kp;
CtrlPara = [Dlt_a,e_now, kp, tau_i, tau_d ,GMVCtemp, eC_SlidSum, grd_K'];
% CtrlPara = [Dlt_a, e_now, K(1), K(2), K(3), GMVCtemp, e_SlidSum, grd_K'];
%% 另外两个通道使用滞后网络校正，双环控制

%%偏航通道控制 角度beta，角速度r  
                        %%%%%%%%%%% 外环，角度控制, 使用单独地外环增益kout%%%%%%%%%%%
e_bta = Eror_ang(2);   %角度控制误差
r_exp = e_bta*Kouty;  %外环控制器，结果作为内环的输入期望
                        %%%%%%%%%%% 内环角速度控制,使用滞后校正网络 %%%%%%%%%%%
e_r = r_exp - r_act;  %角速度控制误差
u_r = e_r*Kinyt;  %先乘以内环增益
u_r_lst = u_lst(2);  Dlt_b_lst = Dlt_lst(2); %读取上次记录值
Dlt_b = DifEquNet(u_r, u_r_lst, Dlt_b_lst, denyz, numyz);  %滞后网络

u_lst(2) = u_r;  Dlt_lst(2) = Dlt_b; %更新本次记录值

%%滚转通道控制 角度mu，角速度p   使用滞后网络校正
                        %%%%%%%%%%% 外环，角度控制, 使用单独地外环增益kout%%%%%%%%%%%
e_mu = Eror_ang(3);   %角度控制误差
p_exp = e_mu*Koutr;  %外环控制器，结果作为内环的输入期望
                        %%%%%%%%%%% 内环角速度控制,使用滞后校正网络 %%%%%%%%%%%
e_p = p_exp - p_act;  %角速度控制误差
u_p = e_p*Kinrt;  %先乘以内环增益
u_p_lst = u_lst(3);  Dlt_m_lst = Dlt_lst(3); %读取上次记录值
Dlt_m = DifEquNet(u_p, u_p_lst, Dlt_m_lst, denrz, numrz);  %滞后网络

u_lst(3) = u_p;  Dlt_lst(3) = Dlt_m; %更新本次记录值

%% 参数传递
Eror_omg(1) = 0;   Eror_omg(2) = e_r;   Eror_omg(3) = e_p;
DeltaE(1) = Dlt_a;   DeltaE(2) = Dlt_b;   DeltaE(3) = Dlt_m;
end 

%% 一阶网络的Z变换的差分方程形式
%输入参量：当前网络输入，前一步输入，前一步输出，脉冲传函的分子分母
%输出参量：当前输出
function [Yout] = DifEquNet(U, U_lst, Y_lst, denz, numz)

Yout = - denz(2)*Y_lst + numz(1)*U + numz(2)*U_lst;

end