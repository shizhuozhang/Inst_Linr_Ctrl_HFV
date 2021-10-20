%%�趨�Ƶ�ָ��
%�������Ϊ:��ǰʱ��T
%�������Ϊ����ǰʱ�̵���̬��ָ��������̬��(alpha,beta,mu)��
function ComdAng = LoadCommand(Time) %���еĽǶȵ�λΪrad

global ctrl_case; %%ѡ��������Σ���Ϊ�����ɶȷ���������ɶȷ���
global Signal_Type;

% global sim_case; %%ѡ��������Σ���Ϊ�����ɶȷ���������ɶȷ���
global Rad2Deg;  %%����ת��Ϊ��
global ComdAng_lst; %��¼��һ�εĽǶ�
global InAng_lst; %��¼��һ�ε�����Ƕ�
Com_beta = 0;    %�໬ʼ���趨Ϊ0
Filter_flag =0;
if Time==0
    if ctrl_case ==1  
        ComdAng_lst = [0,0]/Rad2Deg; %��ʼ������
        InAng_lst = [0,0]/Rad2Deg; %��ʼ������
    else
         ComdAng_lst = [0,0]; %��ʼ������   
         InAng_lst = [0,0]; %��ʼ������
    end
end

%% 100sȫ�������� �Լ����о���Ƶ��Ƶ�ָ��
if ctrl_case ==1  
    if (Time >= 0)&&(Time <= 1)   
        In_alpha = 0/Rad2Deg; %����ָ��
        In_mu = 0/Rad2Deg;    %����ָ��
    elseif (Time > 1)&&(Time <= 10) 
        In_alpha = (2*(Time-1))/Rad2Deg; %����ָ�� б��Ϊ-2��б��ָ��
        In_mu = 0;    %����ָ��
    elseif (Time > 10)&&(Time <= 25) 
        In_alpha = 18/Rad2Deg; %����ָ��
        In_mu = 0;    %����ָ��        
    elseif (Time > 25)&&(Time <= 35) 
        In_alpha = (-Time + 43)/Rad2Deg; %����ָ�� б��Ϊ1��б��ָ��
        In_mu = 0;    %����ָ��        
    elseif (Time > 35)&&(Time <= 50) 
        In_alpha = 8/Rad2Deg; %����ָ��
        In_mu = 0;    %����ָ��     
    elseif (Time > 50)&&(Time <= 64) 
        In_alpha = 8/Rad2Deg; %����ָ��
        In_mu = (-Time + 50)/Rad2Deg;    %����ָ��        
     elseif (Time > 64)&&(Time <= 80) 
        In_alpha = 8/Rad2Deg; %����ָ��
        In_mu = -14/Rad2Deg; %����ָ�� б��Ϊ1.2��б��ָ��            
    elseif (Time > 80)&&(Time <= 90) 
        In_alpha = 8/Rad2Deg; %����ָ��
        In_mu = (2*Time -174)/Rad2Deg; %����ָ�� б��Ϊ1.2��б��ָ��
    elseif (Time > 90)&&(Time <= 100) 
        In_alpha = 8/Rad2Deg; %����ָ��
        In_mu = 6/Rad2Deg; %����ָ�� б��Ϊ1.2��б��ָ��
    else
        In_alpha = 0; %����ָ��
        In_mu = 0;    %����ָ��       
    end

  ComdAng(1) = In_alpha; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;    
%% �������   �����Ľ�Ծָ�� ����һ��һ�׹����˲�����
else
    In_mu = 0;    %����ָ��
    if Signal_Type==11  %%��ֵ�仯�������ź�
        In_alpha = (2 + 0.3*Time)* sin(4* pi* Time/10)/Rad2Deg;   %%Ƶ��1
    elseif Signal_Type==1  %%��ֵ�仯�������ź�
        In_alpha = (2 + 0.3*Time)* sin(3* pi* Time/10)/Rad2Deg;   %%Ƶ��1
    elseif Signal_Type==2 %%��ֵ�仯�������ź�
        In_alpha = (2 + 0.3*Time)* sin(2* pi* Time/10)/Rad2Deg;   %%Ƶ��2
    elseif Signal_Type==3 %%��ֵ�仯�������ź�
        In_alpha = (2 + 0.3*Time)* sin( pi* Time/10)/Rad2Deg;   %%Ƶ��3
    elseif Signal_Type==4  %%��ֵ�仯�������ź�
        In_alpha = (2 + 0.3*Time)* sin( 0.5*pi* Time/10)/Rad2Deg;   %%Ƶ��4
    elseif Signal_Type==5 %%��ֵ�仯�ķ����ź�
        In_alpha = (2 + 0.15*Time)* sign(sin( 3*pi* Time/10))/Rad2Deg;   %%Ƶ��1
        Filter_flag =1;
    elseif Signal_Type==6  %%��ֵ�仯�ķ����ź�
        In_alpha = (2 + 0.15*Time)* sign(sin( 2*pi* Time/10))/Rad2Deg;   %%Ƶ��2
        Filter_flag =1;
     elseif Signal_Type==7 %%��ֵ�仯�ķ����ź�
        In_alpha = (2 + 0.15*Time)* sign(sin( 1*pi* Time/10))/Rad2Deg;   %%Ƶ��3
        Filter_flag =1;
    elseif Signal_Type==8 %%��ֵ�仯�ķ����ź�
        In_alpha = (2 + 0.15*Time)* sign(sin( 0.5*pi* Time/10))/Rad2Deg;   %%Ƶ��4
        Filter_flag =1;
    elseif Signal_Type==9 
        if (Time >= 0)&&(Time <= 20)   
            In_alpha = 10* sin(1.5* pi* Time/10)/Rad2Deg; %����ָ��
        else
            In_alpha = 5* sign(sin(0.8*pi* Time/10))/Rad2Deg;   %%Ƶ��4
        end         
        Filter_flag =1;
    else
        In_alpha = 0; %����ָ��          
    end
       
% % % % % %    ָ���һ���˲����ڣ���������ɵ��ź� ʱ�䳣��0.4s
    if Filter_flag ==1
        Com_alpha_1 = 0.9876*ComdAng_lst(1) + 0.0124* InAng_lst(1); %���ƣ�����0.05     
        InAng_lst(1) = In_alpha;  
        ComdAng_lst(1) = Com_alpha_1;         
        ComdAng(1) = Com_alpha_1; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;   
    else
        ComdAng(1) = In_alpha; ComdAng(2) = Com_beta; ComdAng(3) = In_mu;    
    end
end
   
end

