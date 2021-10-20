%%网络在线训练，梯度法加改进
%%即时线性化，返回线性化参数
function [NNpara] = NN_Idtf_InstLnr( DataPack)
%% 训练数据赋值
% %%% DataPack的定义 = [DataSet_1; DataSet_now; DataSet_lst1; DataSet_lst2];
u_now = DataPack(2,1);
u_lst1 = DataPack(3,1);
u_lst2 = DataPack(4,1);

y_1    = DataPack(1,2);  %%攻角作为输出
y_now  = DataPack(2,2);
y_lst1 = DataPack(3,2);
y_lst2 = DataPack(4,2);

% y_1    = DataPack(1,3);  %%俯仰角速率作为输出
% y_now  = DataPack(2,3);
% y_lst1 = DataPack(3,3);
% y_lst2 = DataPack(4,3);
Pn = [u_lst1, u_now, y_lst1, y_now]';  %%使用二阶模型的输入数据  注意应该是一个列向量
y_act = y_1;  %%实际的下一步输出

%% BP网络参数初始化
global step;     %%记录仿真步数,用于增益调度
global IdtfNN;  %%神经网络全局变量
global dwo_lst;
global dbo_lst;
global dwh_lst;
global dbh_lst;
global gradt_wo_lst;
global gradt_wh_lst;
global LrnRateH;  %学习速率，用于自适应变学习速率
global LrnRateO;  %学习速率，用于自适应变学习速率
global LrnRate;  %学习速率，用于自适应变学习速率
global A_B_bias_lst;

global SlidWd_eI;  %%窗口缓存区，用于计算窗口误差积分
global eI_SlidSum;  %%窗口误差积分
global Num_Count2;  %计数

% Time=0;
% Pn = [0, 0, 0, 0]';  %%使用二阶模型的输入数据  注意应该是一个列向量
% y_act = 0.001;  %%实际的下一步输出
if step==1
    load('Net2O_alp.mat','net');
    IdtfNN = net;  %%离线训练得到的网络作为在线训练的初值
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
gammac = 0.5;  %%学习速率的自适应变化速率 需要调
mc = 0.9;  %动量因子

%% 计算网络输出，对非线性系统进行辨识  
% y_hat1 = sim(IdtfNN,Pn);  %直接利用函数计算网络的预测输出结果
%%%%正向计算网络输出
w_h = IdtfNN.iw{1,1};  b_h = IdtfNN.b{1};  w_o = IdtfNN.lw{2,1};  b_o= IdtfNN.b{2};
m_h = w_h* Pn + b_h;   
Z_h = tansig( m_h );
dZ_h = 1- Z_h .* Z_h;  %%求导
y_hat = purelin( w_o* Z_h + b_o);

e_idtf = y_act - y_hat;    % 辨识误差
NNpara(1,1) = e_idtf;

 %% 计算窗口误差积分，用于判断是否应该调整权值    
if(rem(step,10)==0)  %%每0.05秒更新一个数据，
    Num_Count2 = Num_Count2 +1;   
    x = rem(Num_Count2,10);  %%%窗口设置为0.5S，一共有10个数据
    if (x==0)
        x = 10;  
    end
    SlidWd_eI(1,x) = abs(e_idtf);  %%用这种方法滑动窗口，有点取巧
    eI_SlidSum = sum(SlidWd_eI);  %%累加求和
end
%% 计算即时线性化所得的线性参数
Nh = length( w_o);

if (eI_SlidSum <= 8)   %%辨识误差较小时计算当前线性化系数，否则沿用之前的系数
    a1 =0; a2=0; b1 =0; b2=0;
    for j=1:Nh
        a1 = a1 - w_o(1,j)*w_h(j,1)*dZ_h(j);
        a2 = a2 - w_o(1,j)*w_h(j,2)*dZ_h(j);
        b1 = b1 + w_o(1,j)*w_h(j,3)*dZ_h(j);
        b2 = b2 + w_o(1,j)*w_h(j,4)*dZ_h(j);
    end
    bias = y_act + a1*y_now + a2*y_lst1 - b1*u_now - b2*u_lst1;  %%这个余项的计算要再核实？？
    
    A_B_bias = 0.3*[a1, a2, b1, b2, bias] + 0.7*A_B_bias_lst;   %%加一个低通滤波处理，防止信号突变尖刺的出现
    A_B_bias_lst = A_B_bias;  
%     UDP_flag = 0; %%更新完参数，标志位清零
else
     A_B_bias = A_B_bias_lst ;
end
NNpara(1, 2:6) = A_B_bias;
NNpara(1, 7:10) = Pn';  
NNpara(1,11) = eI_SlidSum;
%% 网络的在线训练
                        %%%%%%下面网络的计算全都是用向量形式进行计算 注意%%%%%%%%%%
%   if (eI_SlidSum > 0.5)  %%在线训练目标设定为0.1度    3-5之间                    
%     %%%误差的反向传播
%     delt_o = e_idtf * 1; %%由输出层反传的误差
%     delt_h = delt_o * w_o' .* dZ_h; %% 由隐含层反传的误差
% 
%     %%%%计算梯度
%     gradt_wo = - delt_o* Z_h;    
%     gradt_wh = - delt_h* Pn';  %%结果是一个矩阵
%     
%     %%%%权值修正  加入动量项，    
% %     %%自适应学习率 由于固定学习速率的效果还不错，够用，没有再调自适应学习率
% %     dw_o = - LrnRateO .* gradt_wo';   db_o = LrnRateO .* delt_o;
% %     LrnRateH_Mt = diag(LrnRateH,0);   LrnRateH_Mt(LrnRateH_Mt==0)=1;
% %     dw_h = - LrnRateH_Mt * gradt_wh;  db_h = LrnRateH .* delt_h;
% %     %%%%计算下一步自适应学习速率
% %     LrnRateO = LrnRateO + gammac * gradt_wo_lst' * gradt_wo;  %%结果为一个实数
% %     dLrnRateH = zeros(H_num,1);
% %     for j=1:Nh
% %         dLrnRateH(j,1) = gradt_wh_lst(j,:) * gradt_wh(j,:)';
% %     end  
% %     LrnRateH = LrnRateH + gammac * dLrnRateH;   %%结果为9*1的向量
%     
%     %%使用固定学习速率
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