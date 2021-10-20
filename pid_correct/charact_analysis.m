clear all;
close all;

 load LnrPara_2_0_1.mat
% % % cp = [c1p, c2p, c3p]; bp = [b2p, b1p, b3p];  
% % % cy = [c1y, c2y, c3y]; by = [b2y, b1y, b3y];  
% % % cr = [c1r, c2r, c3r]; br = [b2r, b1r, b3r]; 
% % % LnrPara = [cp, bp, cy, by, cr, br];  %合成一个总的系数矩阵，用的时候再分解
% 
T = LnrPara_std(:,1);  %用不到这个时间
c1p = LnrPara_std(:,2); c3p = LnrPara_std(:,4);  c2p = 1;%c2p = LnrPara_std(:,3); 
b2p = LnrPara_std(:,5); b1p = LnrPara_std(:,6); b3p = LnrPara_std(:,7); 
c1y = LnrPara_std(:,8); c2y = LnrPara_std(:,9); c3y = LnrPara_std(:,10);
b2y = LnrPara_std(:,11); b1y = LnrPara_std(:,12); b3y = LnrPara_std(:,13);
c1r = LnrPara_std(:,14); c2r = LnrPara_std(:,15); c3r =0; % c3r = LnrPara_std(:,16);
b1r = LnrPara_std(:,18); b3r = LnrPara_std(:,19); b2r =0; % b2r = LnrPara_std(:,17);

FP_abs = [1,10,20,30,45,60,80,99];%待求解的特征点 ，使用的是绝对时刻
 k=1; %用于循环的变量，取值 1-8,%1,10,20,30,45,60,80,100;
% FP_k = find(T == FP_abs(k));

Kin_p = -1500*[5, 4.9, 5.2, 5.8, 7, 7.3, 6, 4.1];  %%%俯仰通道内环增益  %%%对于俯仰通道 2,3,4,5这几个点相似，1,6有细微差别，7，8差别最大
Kin_y =  1500*[6, 6.2, 6.4, 7.2, 8.5, 8.8, 7.2, 4.8];  %%%偏航通道内环增益  %%%对于偏航通道 1,2,3,4,5这几个点相似，6，7，8差别较大
Kin_r = -1000*[3.2, 2.6, 2.6, 3.2, 4.2, 4.3, 3.5, 2.4];  %%%滚转通道内环增益  %%%对于滚转通道 1,2,3,4,5这几个点相似，6，7，8差别较大
%俯仰偏航pm>=55deg,44rad/s
%滚转pm>=80deg,33rad/s

Kout_p = 22; %%1,3,6,9,12
Kout_y = -4; %待调节的内环增益
Kout_r = 4;

% Gm_in1 = zeros(1,12); Gm_in = zeros(1,12); Pm_in = zeros(1,12); Wcg_in =zeros(1,12); Wcp_in =zeros(1,12);Buffer=zeros(1,200);
Gm_gb = zeros(1,29); Pm_gb = zeros(1,29); Wcg_gb = zeros(1,29); Wcp_gb = zeros(1,29); 
count=0;

Channel = 1;%1俯仰，2偏航，3滚转
kp = -5000;  %%注意在程序里用deg为单位时，有50多倍的缩放
tau_i = 5;
tau_d = 0.2; %% -3000,0.1,0.2 %% -1000,0.4,0.3   
%% 飞行器本体模型,即控制框图中各独立组件的模型

for k= 8
    count = count+1;
    
    FP_k = find(T == FP_abs(k));
    
    if(Channel==1)  %俯仰通道
        c1 = c1p(FP_k);    c2 = c2p;    c3 = c3p(FP_k);
        b1 = b1p(FP_k);    b2 = b2p(FP_k);    b3 = b3p(FP_k);  
        
%           %%俯仰通道的校正网络  滞后网络
%         nump = [0.0289,1];  %[0.0577,0.6928],[0.1924,0.231]这组参数也可以用
%         denp = [0.3464,1];  %
%         Gp = tf(nump,denp); 
%         
%         Kin = Kin_p(k); %待调节的内环增益-1000至-4000
%         
%         Kout = Kout_p;
%         
%     elseif(Channel==2)  %偏航通道
%         c1 = c1y(FP_k);    c2 = c2y(FP_k);    c3 = c3y(FP_k);
%         b1 = b1y(FP_k);    b2 = b2y(FP_k);    b3 = b3y(FP_k);    
%         
%        %%%%偏航通道的校正网络  滞后网络
%         nump = [0.0289,1];%0.0577这组参数也可以用
%         denp = [0.3464,1];%0.6928
%         Gp = tf(nump,denp); 
%         
%         Kin = Kin_y(k); %待调节的内环增益800至4000
%         Kout = Kout_y;
%     elseif(Channel==3)  %滚转通道
%         c1 = c1r(FP_k);    c2 = c2r(FP_k);    c3 = 0;
%         b1 = b1r(FP_k);    b2 = 0;           b3 = b3r(FP_k);    
%                   
%         %%滚转通道的校正网络  滞后网络
% %         nump = [0.2203,1];%[0.3525,11.372][0.2938,9.4765]这组参数也可以用
% %         denp = [7.1075,1];%
%         nump = [0.1763,1];%这组参数也可以用
%         denp = [5.686,1];%
%         Gp = tf(nump,denp); 
%          
%         Kin = Kin_r(k); %待调节的内环增益-500至-1500
%         Kout = Kout_r;
    end
     
    %%pid传递函数
    k1 = kp * tau_d; k2 = kp; k3 = kp / tau_i;
    nump = [k1, k2, k3];  
    denp = [1,0];    
    Gc =  tf(nump,denp);
    
    %角度对舵的传递函数
    num = [c3, (b3*c2 - b1*c3)];
    den = [1, -(b1+c1), (c1*b1 -c2*b2)];
    Gad = tf(num,den);

    Gx = Gc * Gad;
    GxCLS = feedback(Gx,1);
     %%%角速度本体特性
%     figure(1);
%      margin(Gx);
% %      axis([0.001 100]);
%      grid on;
%       hold on;
      
     figure(2);      
    step(GxCLS,5);
    grid on;
    hold on  


%     %角速度对舵的传递函数        
%     num = [b3, (b2*c3 - c1*b3)];
%     den = [1, -(b1+c1), (c1*b1 -c2*b2)];
%     Gwd = tf(num,den);
% 
%     %角度对角速度的传递函数
%     num = [c3, (b3*c2 - b1*c3)];
%     den= [b3, (b2*c3 - c1*b3)];
%     Gaw = tf(num,den);
%     
%     Ga = Kin*Gp*Gwd;    % 内环角速度本体的开环传递函数
%     % 内环闭环传递函数
%      Gin = feedback(Ga,1);
%       % 外环闭环传递函数
%      Gb = Kout*Gin*Gaw;  %外环开环传递函数        
%      Gout = feedback(Gb,1);%外环闭环传递函数 
% %       figure(4);
% %      step(Gout,3);
% %      axis([0, 3, 0, 1.3]);
% %      grid on;
% %      hold on
end



%         %四个参数分别为：幅值裕度（单位不是dB,要取20log）；相角裕度（单位：度）；剪切（截止）频率和-180度穿越频率（单位rad/s）
%         %注意开环截止频率和闭环带宽正相关，但大小并不相同
%         %该函数只计算值不画图
%         [Gm_in1(k),Pm_in(k),Wcg_in(k),Wcp_in(k)]=margin(Ga); %开环传函的福相特性，得到裕度大小
%         Gm_in(k) = 20*log10(Gm_in1(k));  %转换为dB为单位

%% 绘制性能和外环增益变化情况
%  gain = 2:1:25;
%  Pm=Pm_gb(1,1:24);
%  Wcp=Wcp_gb(1,1:24);
%  figure(10)
%  [AX,H1,H2]=plotyy(gain, Pm,gain, Wcp); 
% 
% set(AX,'FontSize',13,'FontName','Times New Roman')%设置x轴、左y轴、右y轴刻度字号和字型
% set(AX(1),'Xcolor','k','Ycolor','k')%设置x轴、左y轴刻度字体为黑色；
% set(AX(2),'Xcolor','k','Ycolor','r')%设置x轴、右y轴刻度字体为黑色；
% set(AX,'Xlim',[0 25],'xtick',[0:5:25])%设置x轴数据范围（207.5到217.1），刻度显示（208，209,210...217）
% set(AX(1),'ylim',[50 90],'ytick',[50:5:90])%设置左y轴数据范围（0到0.5），刻度显示（0,0.1,0.2...0.5）
% set(AX(2),'ylim',[0,40],'ytick',[0:5:40])%设置右y轴数据范围（0到3），刻度显示（0,1,2,3）
% set(H1,'Linestyle','-','color','k','Linewidth',2);%设置第一条曲线的线型、颜色、粗细
% set(H2,'Linestyle','-.','color','r','Linewidth',2);%设置第二条曲线的线型、颜色、粗细
% set(get(AX(1),'Ylabel'),'string','Pm (deg)','FontSize',13,'FontName','Times New Roman');%设置左y轴标题字号字型
% set(get(AX(2),'Ylabel'),'string','Wcp (rad/s)','FontSize',13,'FontName','Times New Roman');%设置右y轴标题字号字型
% set(get(AX(1),'Xlabel'),'string','Kout','FontSize',13,'FontName','Times New Roman');%设置x坐标标题字体大小，字型
% grid on;
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);




% [X,Y,Z]=griddata(Pm,Wcp,gain,linspace(min(Pm),max(Pm))',linspace(min(Wcp),max(Wcp)),'v4');
% % % pcolor(X,Y,Z);shading interp;legend('pxolor')%伪彩色图
% figure(11);
% contourf(X,Y,Z,'showText','on');legend('Kout');%等高线图
% xlabel('Pm','FontSize',13);
% ylabel('Wcp','FontSize',13);
% zlabel('Kout','FontSize',13);
% % axis([0 10 -3 1.5])
% % % % h1=legend('PPSMC','SSMC','DSMC','Location','SouthEast');
% % % % set(h1,'box','off');
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
