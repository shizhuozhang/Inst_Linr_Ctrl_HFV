%%��������ѵ�����ݶȷ��ӸĽ�
%%��ʱ���Ի����������Ի�����
function [NNpara] = NN_Idtf_InstLnr( DataPack)
%% ѵ�����ݸ�ֵ
% %%% DataPack�Ķ��� = [DataSet_1; DataSet_now; DataSet_lst1; DataSet_lst2];
u_now = DataPack(2,1);
u_lst1 = DataPack(3,1);
u_lst2 = DataPack(4,1);

y_1    = DataPack(1,2);  %%������Ϊ���
y_now  = DataPack(2,2);
y_lst1 = DataPack(3,2);
y_lst2 = DataPack(4,2);

% y_1    = DataPack(1,3);  %%������������Ϊ���
% y_now  = DataPack(2,3);
% y_lst1 = DataPack(3,3);
% y_lst2 = DataPack(4,3);
Pn = [u_lst1, u_now, y_lst1, y_now]';  %%ʹ�ö���ģ�͵���������  ע��Ӧ����һ��������
y_act = y_1;  %%ʵ�ʵ���һ�����

%% BP���������ʼ��
global step;     %%��¼���沽��,�����������
global IdtfNN;  %%������ȫ�ֱ���
global dwo_lst;
global dbo_lst;
global dwh_lst;
global dbh_lst;
global gradt_wo_lst;
global gradt_wh_lst;
global LrnRateH;  %ѧϰ���ʣ���������Ӧ��ѧϰ����
global LrnRateO;  %ѧϰ���ʣ���������Ӧ��ѧϰ����
global LrnRate;  %ѧϰ���ʣ���������Ӧ��ѧϰ����
global A_B_bias_lst;

global SlidWd_eI;  %%���ڻ����������ڼ��㴰��������
global eI_SlidSum;  %%����������
global Num_Count2;  %����

% Time=0;
% Pn = [0, 0, 0, 0]';  %%ʹ�ö���ģ�͵���������  ע��Ӧ����һ��������
% y_act = 0.001;  %%ʵ�ʵ���һ�����
if step==1
    load('Net2O_alp.mat','net');
    IdtfNN = net;  %%����ѵ���õ���������Ϊ����ѵ���ĳ�ֵ
    [H_num, R_num] = size(net.iw{1,1});
    [O_num,~] = size(net.lw{2,1});
    dwo_lst = zeros(O_num,H_num);
    dbo_lst = zeros(O_num,1);
    dwh_lst = zeros(H_num, R_num);
    dbh_lst = zeros(H_num,1);
    gradt_wo_lst = zeros(H_num,O_num);
    gradt_wh_lst = zeros(H_num, R_num);
    LrnRate = 0.05;
    LrnRateO = 0.01; 
    LrnRateH = 0.01* ones(H_num,1); 
    A_B_bias_lst = zeros(1,5);
    
    SlidWd_eI = zeros(1,10);
    eI_SlidSum = 0;
    Num_Count2 = 0;
end
gammac = 0.5;  %%ѧϰ���ʵ�����Ӧ�仯���� ��Ҫ��
mc = 0.9;  %��������

%% ��������������Է�����ϵͳ���б�ʶ  
% y_hat1 = sim(IdtfNN,Pn);  %ֱ�����ú������������Ԥ��������
%%%%��������������
w_h = IdtfNN.iw{1,1};  b_h = IdtfNN.b{1};  w_o = IdtfNN.lw{2,1};  b_o= IdtfNN.b{2};
m_h = w_h* Pn + b_h;   
Z_h = tansig( m_h );
dZ_h = 1- Z_h .* Z_h;  %%��
y_hat = purelin( w_o* Z_h + b_o);

e_idtf = y_act - y_hat;    % ��ʶ���
NNpara(1,1) = e_idtf;

 %% ���㴰�������֣������ж��Ƿ�Ӧ�õ���Ȩֵ    
if(rem(step,10)==0)  %%ÿ0.05�����һ�����ݣ�
    Num_Count2 = Num_Count2 +1;   
    x = rem(Num_Count2,10);  %%%��������Ϊ0.5S��һ����10������
    if (x==0)
        x = 10;  
    end
    SlidWd_eI(1,x) = abs(e_idtf);  %%�����ַ����������ڣ��е�ȡ��
    eI_SlidSum = sum(SlidWd_eI);  %%�ۼ����
end
%% ���㼴ʱ���Ի����õ����Բ���
Nh = length( w_o);

if (eI_SlidSum <= 8)   %%��ʶ����Сʱ���㵱ǰ���Ի�ϵ������������֮ǰ��ϵ��
    a1 =0; a2=0; b1 =0; b2=0;
    for j=1:Nh
        a1 = a1 - w_o(1,j)*w_h(j,1)*dZ_h(j);
        a2 = a2 - w_o(1,j)*w_h(j,2)*dZ_h(j);
        b1 = b1 + w_o(1,j)*w_h(j,3)*dZ_h(j);
        b2 = b2 + w_o(1,j)*w_h(j,4)*dZ_h(j);
    end
    bias = y_act + a1*y_now + a2*y_lst1 - b1*u_now - b2*u_lst1;  %%�������ļ���Ҫ�ٺ�ʵ����
    
    A_B_bias = 0.3*[a1, a2, b1, b2, bias] + 0.7*A_B_bias_lst;   %%��һ����ͨ�˲�������ֹ�ź�ͻ���̵ĳ���
    A_B_bias_lst = A_B_bias;  
%     UDP_flag = 0; %%�������������־λ����
else
     A_B_bias = A_B_bias_lst ;
end
NNpara(1, 2:6) = A_B_bias;
NNpara(1, 7:10) = Pn';  
NNpara(1,11) = eI_SlidSum;
%% ���������ѵ��
                        %%%%%%��������ļ���ȫ������������ʽ���м��� ע��%%%%%%%%%%
%   if (eI_SlidSum > 0.5)  %%����ѵ��Ŀ���趨Ϊ0.1��    3-5֮��                    
%     %%%���ķ��򴫲�
%     delt_o = e_idtf * 1; %%������㷴�������
%     delt_h = delt_o * w_o' .* dZ_h; %% �������㷴�������
% 
%     %%%%�����ݶ�
%     gradt_wo = - delt_o* Z_h;    
%     gradt_wh = - delt_h* Pn';  %%�����һ������
%     
%     %%%%Ȩֵ����  ���붯���    
% %     %%����Ӧѧϰ�� ���ڹ̶�ѧϰ���ʵ�Ч�����������ã�û���ٵ�����Ӧѧϰ��
% %     dw_o = - LrnRateO .* gradt_wo';   db_o = LrnRateO .* delt_o;
% %     LrnRateH_Mt = diag(LrnRateH,0);   LrnRateH_Mt(LrnRateH_Mt==0)=1;
% %     dw_h = - LrnRateH_Mt * gradt_wh;  db_h = LrnRateH .* delt_h;
% %     %%%%������һ������Ӧѧϰ����
% %     LrnRateO = LrnRateO + gammac * gradt_wo_lst' * gradt_wo;  %%���Ϊһ��ʵ��
% %     dLrnRateH = zeros(H_num,1);
% %     for j=1:Nh
% %         dLrnRateH(j,1) = gradt_wh_lst(j,:) * gradt_wh(j,:)';
% %     end  
% %     LrnRateH = LrnRateH + gammac * dLrnRateH;   %%���Ϊ9*1������
%     
%     %%ʹ�ù̶�ѧϰ����
%     dw_o = - LrnRate .* gradt_wo';   db_o = LrnRate .* delt_o;
%     dw_h = - LrnRate * gradt_wh;  db_h = LrnRate .* delt_h;
%     
%     IdtfNN.iw{1,1} = w_h + (1 - mc) * dw_h + mc * dwh_lst;
%     IdtfNN.b{1}    = b_h + (1 - mc) * db_h + mc * dbh_lst;
%     IdtfNN.lw{2,1} = w_o + (1 - mc) * dw_o + mc * dwo_lst;
%     IdtfNN.b{2}    = b_o + (1 - mc) * db_o + mc * dbo_lst;
%         
%     dwo_lst = dw_o;   dbo_lst = db_o;  dwh_lst = dw_h;   dbh_lst = db_h;
%     gradt_wo_lst = gradt_wo;    gradt_wh_lst = gradt_wh;
%     
%  end

 end