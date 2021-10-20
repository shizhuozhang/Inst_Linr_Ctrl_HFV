%%普通滑模控制器
%双环控制，输入：指令角，实际角，角速度
%输出：角度控制误差，角速度控制误差，角加速度，等效舵偏
% alpha = FltSdt_F(4);   beta = FltSdt_F(5);   mu = FltSdt_F(6); %构成气动角矢量 AeroAng = [alpha, beta, mu];
% q = FltSdt_F(7);   r = FltSdt_F(8);   p = FltSdt_F(9); %构成角速率矢量 Omega = [q ,r, p];
%Cmd_alp = CtrlSdt_B(1);  Cmd_bet = CtrlSdt_B(2);    Cmd_mu = CtrlSdt_B(3); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
%Ero_pit = CtrlSdt_P(1);  Ero_yaw = CtrlSdt_P(2);    Ero_rol = CtrlSdt_P(3); %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
%Ero_qw = CtrlSdt_P(4);  Ero_rw = CtrlSdt_P(5);    Ero_pw = CtrlSdt_P(6); %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
%Acc_pit = CtrlSdt_P(7);  Acc_yaw = CtrlSdt_P(8);    Acc_rol = CtrlSdt_P(9); %构成角加速度矢量 Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
%Dlt_alp = CtrlSdt_P(10);  Dlt_bet = CtrlSdt_P(11);    Dlt_mu = CtrlSdt_P(12); %构成等效舵偏矢量 DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
function [Eror_ang, Delta,SMCData] = Ctrller_SMC_Sg(AfnPara, CmdAng, AeroAng, Omega)

global step;     %%记录仿真步数,用于增益调度
global h_step;   %%仿真步长
%新定义的全局变量，为了记录中间值而不被清除
global CmdAng_lst;  %记录上次角度，用于求取变化率
global dCmdAng_lst;  %记录上次角速度，用于求取变化率

if step==1
    CmdAng_lst = zeros(3,1); %初始化参数
    dCmdAng_lst = zeros(3,1); %初始化参数
end

%模型参数赋值
%%%% AfnPara = [gs,gf, ff, dst_F, dst_Mx];%%3*(3+3+1+1+1)=3*9
gs = AfnPara(:,1:3);
gf = AfnPara(:,4:6);
ff = AfnPara(:,7);
dst_F = AfnPara(:,8);
dst_M = AfnPara(:,9);
dgs = AfnPara(:,10:12);


%% 带调节的参数
Dstb_est = [0;0;0];
% c = [5; 5; 5];  %角速度控制的积分滑模变量参数 可用参数：20
% k1 = [3; 2; 3];  %趋近律参数 可用参数：10
% k2 = [15; 10; 15];      %可用参数：
% c = [2; 2; 2];  %角速度控制的积分滑模变量参数 可用参数：20
% k1 = [1.5; 1.5; 1.5];  %趋近律参数 可用参数：10
% k2 = [3; 3; 3];      %可用参数：

c = [4; 4; 4];  %角速度控制的积分滑模变量参数 可用参数：20
k1 = [3; 2; 3];  %趋近律参数 可用参数：10
k2 = [6; 4; 6];      %可用参数：
%% 直接使用矢量控制三通道
gx = gs*gf;
fw = gs*ff + dgs*(Omega') + (gs*(Omega')).*c;
Dstb = c.*dst_F + gs*(dst_M + Dstb_est) ;%估计的干扰界

dComAng = (CmdAng' - CmdAng_lst)./h_step; %注意这里使用列向量
ddComAng = (dComAng - dCmdAng_lst)./h_step; %注意这里使用列向量

Eror_ang = (AeroAng' - CmdAng'); 
dEror_ang = gs*(Omega') + dst_F - dComAng;
sigma = c.*Eror_ang + dEror_ang;

%控制律
Uc =  gx\ (c.*dComAng + ddComAng- fw - Dstb - k1.*tanh(sigma/0.8)- k2.*sigma);% 外环控制律，产生角速率期望
%Uc = gf \(dComOmg - ff - c_omg*Eror_omg - Dstb_est_O - k1_omg*sign(sigma_omg)); % 外环控制律，产生角速率期望

CmdAng_lst = CmdAng'; %记录指令角
dCmdAng_lst = dComAng;
%% 得到最终的舵偏  
if abs(Uc(1)) > 20
    Uc(1) = sign(Uc(1))*20;
end
if abs(Uc(2)) > 20
    Uc(2) = sign(Uc(2))*20;
end
if abs(Uc(3)) > 20
    Uc(3) = sign(Uc(3))*20;
end  
Delta = Uc'; %%转成行向量

SMCData= [Eror_ang',dEror_ang',dComAng', ddComAng', sigma',fw',Dstb', Delta];
% SMCData= [sigma_ang', sigma_omg'];
end 
