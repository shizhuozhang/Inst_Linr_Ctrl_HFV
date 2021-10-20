clear all;
close all;

 load LnrPara_2_0_1.mat
% % % cp = [c1p, c2p, c3p]; bp = [b2p, b1p, b3p];  
% % % cy = [c1y, c2y, c3y]; by = [b2y, b1y, b3y];  
% % % cr = [c1r, c2r, c3r]; br = [b2r, b1r, b3r]; 
% % % LnrPara = [cp, bp, cy, by, cr, br];  %�ϳ�һ���ܵ�ϵ�������õ�ʱ���ٷֽ�
% 
T = LnrPara_std(:,1);  %�ò������ʱ��
c1p = LnrPara_std(:,2); c3p = LnrPara_std(:,4);  c2p = 1;%c2p = LnrPara_std(:,3); 
b2p = LnrPara_std(:,5); b1p = LnrPara_std(:,6); b3p = LnrPara_std(:,7); 
c1y = LnrPara_std(:,8); c2y = LnrPara_std(:,9); c3y = LnrPara_std(:,10);
b2y = LnrPara_std(:,11); b1y = LnrPara_std(:,12); b3y = LnrPara_std(:,13);
c1r = LnrPara_std(:,14); c2r = LnrPara_std(:,15); c3r =0; % c3r = LnrPara_std(:,16);
b1r = LnrPara_std(:,18); b3r = LnrPara_std(:,19); b2r =0; % b2r = LnrPara_std(:,17);

FP_abs = [1,10,20,30,45,60,80,99];%������������ ��ʹ�õ��Ǿ���ʱ��
 k=1; %����ѭ���ı�����ȡֵ 1-8,%1,10,20,30,45,60,80,100;
% FP_k = find(T == FP_abs(k));

Kin_p = -1500*[5, 4.9, 5.2, 5.8, 7, 7.3, 6, 4.1];  %%%����ͨ���ڻ�����  %%%���ڸ���ͨ�� 2,3,4,5�⼸�������ƣ�1,6��ϸ΢���7��8������
Kin_y =  1500*[6, 6.2, 6.4, 7.2, 8.5, 8.8, 7.2, 4.8];  %%%ƫ��ͨ���ڻ�����  %%%����ƫ��ͨ�� 1,2,3,4,5�⼸�������ƣ�6��7��8���ϴ�
Kin_r = -1000*[3.2, 2.6, 2.6, 3.2, 4.2, 4.3, 3.5, 2.4];  %%%��תͨ���ڻ�����  %%%���ڹ�תͨ�� 1,2,3,4,5�⼸�������ƣ�6��7��8���ϴ�
%����ƫ��pm>=55deg,44rad/s
%��תpm>=80deg,33rad/s

Kout_p = 22; %%1,3,6,9,12
Kout_y = -4; %�����ڵ��ڻ�����
Kout_r = 4;

% Gm_in1 = zeros(1,12); Gm_in = zeros(1,12); Pm_in = zeros(1,12); Wcg_in =zeros(1,12); Wcp_in =zeros(1,12);Buffer=zeros(1,200);
Gm_gb = zeros(1,29); Pm_gb = zeros(1,29); Wcg_gb = zeros(1,29); Wcp_gb = zeros(1,29); 
count=0;

Channel = 1;%1������2ƫ����3��ת
kp = -5000;  %%ע���ڳ�������degΪ��λʱ����50�౶������
tau_i = 5;
tau_d = 0.2; %% -3000,0.1,0.2 %% -1000,0.4,0.3   
%% ����������ģ��,�����ƿ�ͼ�и����������ģ��

for k= 8
    count = count+1;
    
    FP_k = find(T == FP_abs(k));
    
    if(Channel==1)  %����ͨ��
        c1 = c1p(FP_k);    c2 = c2p;    c3 = c3p(FP_k);
        b1 = b1p(FP_k);    b2 = b2p(FP_k);    b3 = b3p(FP_k);  
        
%           %%����ͨ����У������  �ͺ�����
%         nump = [0.0289,1];  %[0.0577,0.6928],[0.1924,0.231]�������Ҳ������
%         denp = [0.3464,1];  %
%         Gp = tf(nump,denp); 
%         
%         Kin = Kin_p(k); %�����ڵ��ڻ�����-1000��-4000
%         
%         Kout = Kout_p;
%         
%     elseif(Channel==2)  %ƫ��ͨ��
%         c1 = c1y(FP_k);    c2 = c2y(FP_k);    c3 = c3y(FP_k);
%         b1 = b1y(FP_k);    b2 = b2y(FP_k);    b3 = b3y(FP_k);    
%         
%        %%%%ƫ��ͨ����У������  �ͺ�����
%         nump = [0.0289,1];%0.0577�������Ҳ������
%         denp = [0.3464,1];%0.6928
%         Gp = tf(nump,denp); 
%         
%         Kin = Kin_y(k); %�����ڵ��ڻ�����800��4000
%         Kout = Kout_y;
%     elseif(Channel==3)  %��תͨ��
%         c1 = c1r(FP_k);    c2 = c2r(FP_k);    c3 = 0;
%         b1 = b1r(FP_k);    b2 = 0;           b3 = b3r(FP_k);    
%                   
%         %%��תͨ����У������  �ͺ�����
% %         nump = [0.2203,1];%[0.3525,11.372][0.2938,9.4765]�������Ҳ������
% %         denp = [7.1075,1];%
%         nump = [0.1763,1];%�������Ҳ������
%         denp = [5.686,1];%
%         Gp = tf(nump,denp); 
%          
%         Kin = Kin_r(k); %�����ڵ��ڻ�����-500��-1500
%         Kout = Kout_r;
    end
     
    %%pid���ݺ���
    k1 = kp * tau_d; k2 = kp; k3 = kp / tau_i;
    nump = [k1, k2, k3];  
    denp = [1,0];    
    Gc =  tf(nump,denp);
    
    %�ǶȶԶ�Ĵ��ݺ���
    num = [c3, (b3*c2 - b1*c3)];
    den = [1, -(b1+c1), (c1*b1 -c2*b2)];
    Gad = tf(num,den);

    Gx = Gc * Gad;
    GxCLS = feedback(Gx,1);
     %%%���ٶȱ�������
%     figure(1);
%      margin(Gx);
% %      axis([0.001 100]);
%      grid on;
%       hold on;
      
     figure(2);      
    step(GxCLS,5);
    grid on;
    hold on  


%     %���ٶȶԶ�Ĵ��ݺ���        
%     num = [b3, (b2*c3 - c1*b3)];
%     den = [1, -(b1+c1), (c1*b1 -c2*b2)];
%     Gwd = tf(num,den);
% 
%     %�ǶȶԽ��ٶȵĴ��ݺ���
%     num = [c3, (b3*c2 - b1*c3)];
%     den= [b3, (b2*c3 - c1*b3)];
%     Gaw = tf(num,den);
%     
%     Ga = Kin*Gp*Gwd;    % �ڻ����ٶȱ���Ŀ������ݺ���
%     % �ڻ��ջ����ݺ���
%      Gin = feedback(Ga,1);
%       % �⻷�ջ����ݺ���
%      Gb = Kout*Gin*Gaw;  %�⻷�������ݺ���        
%      Gout = feedback(Gb,1);%�⻷�ջ����ݺ��� 
% %       figure(4);
% %      step(Gout,3);
% %      axis([0, 3, 0, 1.3]);
% %      grid on;
% %      hold on
end



%         %�ĸ������ֱ�Ϊ����ֵԣ�ȣ���λ����dB,Ҫȡ20log�������ԣ�ȣ���λ���ȣ������У���ֹ��Ƶ�ʺ�-180�ȴ�ԽƵ�ʣ���λrad/s��
%         %ע�⿪����ֹƵ�ʺͱջ���������أ�����С������ͬ
%         %�ú���ֻ����ֵ����ͼ
%         [Gm_in1(k),Pm_in(k),Wcg_in(k),Wcp_in(k)]=margin(Ga); %���������ĸ������ԣ��õ�ԣ�ȴ�С
%         Gm_in(k) = 20*log10(Gm_in1(k));  %ת��ΪdBΪ��λ

%% �������ܺ��⻷����仯���
%  gain = 2:1:25;
%  Pm=Pm_gb(1,1:24);
%  Wcp=Wcp_gb(1,1:24);
%  figure(10)
%  [AX,H1,H2]=plotyy(gain, Pm,gain, Wcp); 
% 
% set(AX,'FontSize',13,'FontName','Times New Roman')%����x�ᡢ��y�ᡢ��y��̶��ֺź�����
% set(AX(1),'Xcolor','k','Ycolor','k')%����x�ᡢ��y��̶�����Ϊ��ɫ��
% set(AX(2),'Xcolor','k','Ycolor','r')%����x�ᡢ��y��̶�����Ϊ��ɫ��
% set(AX,'Xlim',[0 25],'xtick',[0:5:25])%����x�����ݷ�Χ��207.5��217.1�����̶���ʾ��208��209,210...217��
% set(AX(1),'ylim',[50 90],'ytick',[50:5:90])%������y�����ݷ�Χ��0��0.5�����̶���ʾ��0,0.1,0.2...0.5��
% set(AX(2),'ylim',[0,40],'ytick',[0:5:40])%������y�����ݷ�Χ��0��3�����̶���ʾ��0,1,2,3��
% set(H1,'Linestyle','-','color','k','Linewidth',2);%���õ�һ�����ߵ����͡���ɫ����ϸ
% set(H2,'Linestyle','-.','color','r','Linewidth',2);%���õڶ������ߵ����͡���ɫ����ϸ
% set(get(AX(1),'Ylabel'),'string','Pm (deg)','FontSize',13,'FontName','Times New Roman');%������y������ֺ�����
% set(get(AX(2),'Ylabel'),'string','Wcp (rad/s)','FontSize',13,'FontName','Times New Roman');%������y������ֺ�����
% set(get(AX(1),'Xlabel'),'string','Kout','FontSize',13,'FontName','Times New Roman');%����x������������С������
% grid on;
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);




% [X,Y,Z]=griddata(Pm,Wcp,gain,linspace(min(Pm),max(Pm))',linspace(min(Wcp),max(Wcp)),'v4');
% % % pcolor(X,Y,Z);shading interp;legend('pxolor')%α��ɫͼ
% figure(11);
% contourf(X,Y,Z,'showText','on');legend('Kout');%�ȸ���ͼ
% xlabel('Pm','FontSize',13);
% ylabel('Wcp','FontSize',13);
% zlabel('Kout','FontSize',13);
% % axis([0 10 -3 1.5])
% % % % h1=legend('PPSMC','SSMC','DSMC','Location','SouthEast');
% % % % set(h1,'box','off');
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
