%%У�����������
%˫�����ƣ����룺ָ��ǣ�ʵ�ʽǣ����ٶ�
%������Ƕȿ��������ٶȿ������Ǽ��ٶȣ���Ч��ƫ
% alpha = FltSdt_F(4);   beta = FltSdt_F(5);   mu = FltSdt_F(6); %����������ʸ�� AeroAng = [alpha, beta, mu];
% q = FltSdt_F(7);   r = FltSdt_F(8);   p = FltSdt_F(9); %���ɽ�����ʸ�� Omega = [q ,r, p];
%Cmd_alp = CtrlSdt_B(1);  Cmd_bet = CtrlSdt_B(2);    Cmd_mu = CtrlSdt_B(3); %����ָ��ʸ�� CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
%Ero_pit = CtrlSdt_P(1);  Ero_yaw = CtrlSdt_P(2);    Ero_rol = CtrlSdt_P(3); %���ɿ������ʸ�� Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
%Ero_qw = CtrlSdt_P(4);  Ero_rw = CtrlSdt_P(5);    Ero_pw = CtrlSdt_P(6); %���ɽ��ٶȿ������ʸ�� Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
%Acc_pit = CtrlSdt_P(7);  Acc_yaw = CtrlSdt_P(8);    Acc_rol = CtrlSdt_P(9); %���ɽǼ��ٶ�ʸ�� Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
%Dlt_alp = CtrlSdt_P(10);  Dlt_bet = CtrlSdt_P(11);    Dlt_mu = CtrlSdt_P(12); %���ɵ�Ч��ƫʸ�� DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
function [Eror_ang, Eror_omg, DeltaE, CtrlPara] = Ctrller_GMVCPID(CmdAng, AeroAng, Omega, NNpara)

global h_step;   %%���沽��
global step;     %%��¼���沽��,�����������
global Rad2Deg;  %%����ת��Ϊ��
global UDP_flag;  %%���²����ı�־λ

%�¶����ȫ�ֱ�����Ϊ�˼�¼�м�ֵ���������
global u_lst; %��ͨ�����ڻ������ϴ������¼ֵ
global Dlt_lst; %��ͨ�����ڻ������ϴ������¼ֵ
global Kinp_x; %�ڻ�����������
global Kiny_x; %�ڻ�����������
global Kinr_x; %�ڻ�����������

%����ӦPID��صı���
global e_lst2;
global e_lst1;
global K;
global GMVCtemp; 
global K_init;
global Cof_Mu;
global SlidWd_eC;  %%���ڻ����������ڼ��㴰��������
global eC_SlidSum;  %%����������
global Num_Count1;  %����
global grd_K;

if step==1
    e_lst1 = 0;
    e_lst2 = 0;
    kp = -60;  tau_i = 2.5;  tau_d = 0.2;  %%�����ʼ����������  %%60,2.5,0.2%%40,2.5,0.15
    k1 = kp* (1 + h_step/2/tau_i + tau_d/h_step); 
    k2 = - kp* (1 - h_step/2/tau_i + 2*tau_d/h_step);
    k3 = kp* tau_d/ h_step;
    K =[k1, k2, k3]';
    GMVCtemp = 0;
    K_init = K;
    Cof_Mu = 0.5;  %%����ϵ��
    SlidWd_eC = zeros(1,10);
    eC_SlidSum = 0;
    Num_Count1 = 0;
    grd_K = zeros(3,1);
    
    u_lst = [0,0,0]; %��ʼ������
    Dlt_lst = [0,0,0]; %��ʼ������
    load Kin_para_N.mat    %�ڻ�����'Kinp','Kiny','Kinr'  ��ֵ��Χ0: 0.01: 100;
    Kinp_x = Kinp;
    Kiny_x = Kiny;
    Kinr_x = Kinr;   
end

% Kinpt = Kinp_x(round((step-1)/2) +1); %��ȡ��ֵ���ڻ�����
% Koutp = 5; numpz = [0.08343,-0.06910]; denpz = [1,-0.98567];
Kinyt = Kiny_x(round((step-1)/2) +1); %��ȡ��ֵ���ڻ�����
Kouty = -5; numyz = [0.08343,-0.06910]; denyz = [1,-0.98567];
Kinrt = Kinr_x(round((step-1)/2) +1); %��ȡ��ֵ���ڻ�����
Koutr = 5;  numrz = [0.03101,-0.03013]; denrz = [1,-0.99912];

%% ����״̬
Eror_ang = CmdAng - AeroAng;  %�Ƕȿ���������ָ��Ǽ�ʵ�ʽǶ�
q_act = Omega(1); r_act = Omega(2); p_act = Omega(3); %ʵ�ʽ��ٶ�
%% ����ͨ������ �Ƕ�alpha�����ٶ�q  ʹ��PID���ƽṹ ������С�����������Ӧ�������ŵ�PID���� ��������
                      
e_now = Eror_ang(1) *Rad2Deg;   %�Ƕȿ������  ���ڱ�ʶ����õĵ�λdeg���������ͳһתΪdeg����
E = [e_now, e_lst1, e_lst2]';
 du = E'*K_init; %%����PID���������������
% du = E'*K; %%����PID���������������
Dlt_a = Dlt_lst(1) + du;

e_lst2 = e_lst1;  e_lst1 = e_now;
Dlt_lst(1) = Dlt_a; %���±��μ�¼ֵ

        %%%%���㴰�������֣������ж��Ƿ�Ӧ�ñ��    
if(rem(step,10)==0)  %%ÿ0.05�����һ�����ݣ�
    Num_Count1 = Num_Count1 +1;   
    x = rem(Num_Count1,10);  %%%��������Ϊ0.5S��һ����10������
    if (x==0)
        x = 10;  
    end
    SlidWd_eC(1,x) = abs(e_now);  %%�����ַ����������ڣ��е�ȡ��
    eC_SlidSum = sum(SlidWd_eC);  %%�ۼ����
end
           %%%%���ù�����С�������ſ���ָ��  ����Ӧ��������ͨ��������
CgRt = 0.1;

e_idtf = NNpara(1);   a1 = NNpara(2);  a2 = NNpara(3);  b1 = NNpara(4);  b2 = NNpara(5);  bias = NNpara(6); 
uk_lst1 = NNpara(7);  uk_now = NNpara(8);   yk_lst1 = NNpara(9);  yk_now  = NNpara(10);  

%     if abs(e_idtf) > 0.1
%         Cof_Mu = 0.5*Cof_Mu;
%     elseif abs(e_idtf) < 0.005
%         Cof_Mu = 2*Cof_Mu;
%     end

GMVCtemp = CmdAng(1)*Rad2Deg + a1*yk_now + a2*yk_lst1- b1*uk_lst1 - b2*uk_lst1 - bias;

%%���þ����������Ų��������ǲ����ݶȷ������������
grd_K = (b1*b1 + Cof_Mu).* E*E'*K - GMVCtemp*b1*E;   %%�����ݶ�

if (eC_SlidSum >= 0.5)%&&(UDP_flag==1)   %%%����0.5���������0.1��ʱ,�Ž��в������� 
    dK = - CgRt .* grd_K;
    K = K + dK;
%    UDP_flag = 0; %%�������������־λ����
end

%     cof = GMVCtemp*b1/(b1*b1 + Cof_Mu);
%     tmp1 = E*E';    %%��һ�����ڲ�����С��ǰ�󼸲����������ֵ�����󣬵��½����������������ֻ���������
%     if(tmp1(1,1)==0)||(tmp1(2,2)==0)||(tmp1(3,3)==0) 
%         tmp1 = tmp1 + diag(0.000001*ones(3,1)); 
%     end
%     tmp2 = pinv(tmp1);
%     tmp3 = tmp2 *E;
%     K = cof.* tmp3;  %%���������е��Ƶ��������Ų���            

kp = 0.5* (K(1) - K(2) - 3*K(3));
tau_i = h_step* kp / (K(1) + K(2) + K(3));
tau_d = h_step* K(3) / kp;
CtrlPara = [Dlt_a,e_now, kp, tau_i, tau_d ,GMVCtemp, eC_SlidSum, grd_K'];
% CtrlPara = [Dlt_a, e_now, K(1), K(2), K(3), GMVCtemp, e_SlidSum, grd_K'];
%% ��������ͨ��ʹ���ͺ�����У����˫������

%%ƫ��ͨ������ �Ƕ�beta�����ٶ�r  
                        %%%%%%%%%%% �⻷���Ƕȿ���, ʹ�õ������⻷����kout%%%%%%%%%%%
e_bta = Eror_ang(2);   %�Ƕȿ������
r_exp = e_bta*Kouty;  %�⻷�������������Ϊ�ڻ�����������
                        %%%%%%%%%%% �ڻ����ٶȿ���,ʹ���ͺ�У������ %%%%%%%%%%%
e_r = r_exp - r_act;  %���ٶȿ������
u_r = e_r*Kinyt;  %�ȳ����ڻ�����
u_r_lst = u_lst(2);  Dlt_b_lst = Dlt_lst(2); %��ȡ�ϴμ�¼ֵ
Dlt_b = DifEquNet(u_r, u_r_lst, Dlt_b_lst, denyz, numyz);  %�ͺ�����

u_lst(2) = u_r;  Dlt_lst(2) = Dlt_b; %���±��μ�¼ֵ

%%��תͨ������ �Ƕ�mu�����ٶ�p   ʹ���ͺ�����У��
                        %%%%%%%%%%% �⻷���Ƕȿ���, ʹ�õ������⻷����kout%%%%%%%%%%%
e_mu = Eror_ang(3);   %�Ƕȿ������
p_exp = e_mu*Koutr;  %�⻷�������������Ϊ�ڻ�����������
                        %%%%%%%%%%% �ڻ����ٶȿ���,ʹ���ͺ�У������ %%%%%%%%%%%
e_p = p_exp - p_act;  %���ٶȿ������
u_p = e_p*Kinrt;  %�ȳ����ڻ�����
u_p_lst = u_lst(3);  Dlt_m_lst = Dlt_lst(3); %��ȡ�ϴμ�¼ֵ
Dlt_m = DifEquNet(u_p, u_p_lst, Dlt_m_lst, denrz, numrz);  %�ͺ�����

u_lst(3) = u_p;  Dlt_lst(3) = Dlt_m; %���±��μ�¼ֵ

%% ��������
Eror_omg(1) = 0;   Eror_omg(2) = e_r;   Eror_omg(3) = e_p;
DeltaE(1) = Dlt_a;   DeltaE(2) = Dlt_b;   DeltaE(3) = Dlt_m;
end 

%% һ�������Z�任�Ĳ�ַ�����ʽ
%�����������ǰ�������룬ǰһ�����룬ǰһ����������崫���ķ��ӷ�ĸ
%�����������ǰ���
function [Yout] = DifEquNet(U, U_lst, Y_lst, denz, numz)

Yout = - denz(2)*Y_lst + numz(1)*U + numz(2)*U_lst;

end