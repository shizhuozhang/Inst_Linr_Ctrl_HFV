%%初始化仿真参数
%输入参数：仿真情形和特征点
%输出：初始的飞行状态参数和不确定性设定值
function [FltSdt_S, FltSdt_F] = Parameter_Init(Ft_p)

global ctrl_case; %%选择控制情形，分为全弹道控制和单点控制
global Rad2Deg;  %%弧度转换为度
global Uncertn_flag; %是否引入外部风干扰（随机干扰）
global Uncertn_para;
global Bias_case; %拉偏情况
global Bias_amp;  %拉偏极限幅值%拉偏极限设定，最大0.3
%% 不确定性设定，包括正负极限拉偏和拉丁超立方随机抽样拉偏情况（最大设定30%拉偏）
%不确定向量顺序;    xm0; xJx; xJy; xJz; xRho; xCD; xCY; xCL; xCl; xCm; xCn ; 
Uncertn_para = zeros(1,11); %初始值均设为0
if Uncertn_flag == 1  
    %正负极限拉偏（分为结构参数和气动参数两部分，其中气动参数部分又分为三个控制通道单独研究）
    BCas = Limit_bias(4,Bias_amp/100);
    
    xRho = BCas(Bias_case,1); xm  = BCas(Bias_case,2);  %%共有的拉偏参数，动压和质量
    xJy = BCas(Bias_case,3);  xCm = BCas(Bias_case,4);% 对俯仰相关参数进行拉偏 xRho;xCL; xCm;
    xJx = BCas(Bias_case,3);  xCl = BCas(Bias_case,4);% 对滚转相关气动参数进行拉偏 xRho; xCl;
%     xJz = BCas(Bias_case,3);  xCn = BCas(Bias_case,4);  % 对偏航相关气动参数进行拉偏 xRho; xCY; xCn ;
%     xCD = BCas(Bias_case,3);  xCL = BCas(Bias_case,4);  xCY = BCas(Bias_case,3);   
    xJz = 0;  xCn = 0;  % 对偏航相关气动参数进行拉偏 xRho; xCY; xCn ;
    xCD = 0;  xCL = 0;  xCY = 0;  
    
    Uncertn_para(1) = xm;  Uncertn_para(2) = xJx;   Uncertn_para(3) = xJy;  Uncertn_para(4) = xJz;
    Uncertn_para(5) = xRho; Uncertn_para(6) = xCD; Uncertn_para(7) = xCY;  Uncertn_para(8) = xCL;
    Uncertn_para(9) = xCl; Uncertn_para(10) = xCm;  Uncertn_para(11) = xCn;
else     
    Uncertn_para = zeros(1,11); 
end

%%干扰设定 范围0-0.2  性质和不确定性差不多 但应该是时变的，因此这里不做初始化，在力的计算函数里计算

%% 100s全弹道仿真
if ctrl_case==1  %除了高度、速度外其他量全部置零
    X_loc = 0;    Y_loc = 0;    Height = 33000;
    Veloc = 4590;    gamma = 0/Rad2Deg;    chi = 0/Rad2Deg;
    alpha = 0/Rad2Deg;    beta = 0/Rad2Deg;    mu = 0/Rad2Deg;
    q = 0;    r = 0;    p = 0;   
    
%% 单点仿真   根据标称弹道的特征点处的具体状态设定
else  
    switch Ft_p
        case 1
            X_loc = 0;    Y_loc = 0;    Height = 33000;
            Veloc = 4590;    gamma = 0;    chi = 0;
            alpha = 0;    beta = 0;    mu = 0;
            q = 0;    r = 0;    p = 0;  
         case 2
            X_loc = 0;    Y_loc = 0;    Height = 33000;
            Veloc = 4590;    gamma = 0;    chi = 0;
            alpha = 2/Rad2Deg;    beta = 0;    mu = 0/Rad2Deg;
            q = 0;    r = 0;    p = 0;  
        case 3  
            X_loc = 0;    Y_loc = 0;    Height = 325130;
            Veloc = 4559;    gamma = -0.98/Rad2Deg;    chi = 0/Rad2Deg;
            alpha = 14/Rad2Deg;    beta = 0/Rad2Deg;    mu = -2/Rad2Deg;
            q = 0;    r = 0;    p = 0;  
    end 
end

%通过几何关系解欧拉姿态角
AeroAng = [alpha, beta, mu];
Trajct = [Veloc, gamma, chi];
EulerAng = GometrRelat(1, AeroAng ,Trajct);%求解欧拉角
theta = EulerAng(1); psi = EulerAng(2); phi = EulerAng(3);

FltSdt_S(1) = X_loc;   FltSdt_S(2) = Y_loc;   FltSdt_S(3) = Height;
FltSdt_S(4) = Veloc;   FltSdt_S(5) = gamma;   FltSdt_S(6) = chi;

FltSdt_F(1) = theta;   FltSdt_F(2) = psi;   FltSdt_F(3) = phi;
FltSdt_F(4) = alpha;   FltSdt_F(5) = beta;   FltSdt_F(6) = mu;
FltSdt_F(7) = q;   FltSdt_F(8) = r;   FltSdt_F(9) = p;
end
