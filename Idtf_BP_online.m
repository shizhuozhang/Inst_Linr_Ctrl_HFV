% 2018��11��19�� by zhangyaokun
% BP identification
clear all;
close all;
%% BP���������ʼ��
xite=0.30;  %ѧϰ����
alfa=0.05;  %��������

w2=rands(9,1);  %�����ʼ������Ȩֵ
w2_1=w2;w2_2=w2_1;  %����ṹ��4-9-1
w1=rands(4,9);
w1_1=w1;w1_2=w1;
dw1=0*w1;

I=[0,0,0,0,0,0,0,0,0]';
Iout=[0,0,0,0,0,0,0,0,0]';
FI=[0,0,0,0,0,0,0,0,0]';

%% ѵ�����磬�Է�����ϵͳ���б�ʶ
for count=1:100
    x=[0,0,0,0]';%����ĳ�ʼ����Ϊ0
 %   x=[0,0,0]';%����ĳ�ʼ����Ϊ0
    u_1=0;y_1=0;y_2=0;
    
    for  k=1:1:500   %ʹ��500��������ѵ������
        time(k)=k;
        u(k)=sin(2*pi/3*k) + 1/3*sin(2*pi/6*k);
        y(k) = u_1+ y_1 * (2*y_2 + 1) / (1+ y_1^2 + y_2^2);  %������ϵͳ

        for  j=1:1:9
             I(j)=x'*w1(:,j);  
  %           Iout(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));
             Iout(j)=1/(1+exp(-I(j))); %���������������ʹ��sigmod����
        end   
        yn(k)=w2'*Iout;         % �������������
        e(k)=y(k)-yn(k);    % ��ʶ���

        %����Ȩֵ����
        w2=w2_1+(xite*e(k))*Iout+alfa*(w2_1-w2_2);  %���򴫲����ݶȷ��������Ȩֵ����ֵ

        for j=1:1:9
          FI(j)=exp(-I(j))/(1+exp(-I(j)))^2;  %����������ĵ���
%             FI(j)=4/((exp(I(j))+exp(-I(j)))^2);
        end
        for i=1:1:4
           for j=1:1:9
              dw1(i,j)=xite*e(k)*FI(j)*w2(j)*x(i);
           end
        end
        w1=w1_1+dw1+alfa*(w1_1-w1_2);

        for j=1:1:9
           yu=w2(j)*w1(1,j)*FI(j);    %Jacobian
        end
        dyu(k)=yu;

        %����������
        x(1)=u(k);
        x(2)=y(k);
         x(3)=y_1;
         x(4)=y_2;

        w1_2=w1_1;w1_1=w1;
        w2_2=w2_1;w2_1=w2;
        u_1=u(k);
        y_2=y_1;
        y_1=y(k);
    end
end
%% ����ѵ�����������ʾ
figure(1);
plot(time,y,'r',time,yn,'b');
xlabel('times');ylabel('y and yn');
figure(2);
plot(time,e,'r');
xlabel('times');ylabel('error');
figure(3);
plot(time,dyu);
xlabel('times');ylabel('dyu');

%% ʹ���µ����ݲ�������
x=[0,0,0,0]';  %�����ʼ����Ϊ0
u_1=0;y_1=0;y_2=0;  %ϵͳ��ʼ����������³�ʼ��Ϊ0
I=[0,0,0,0,0,0,0]';
Iout=[0,0,0,0,0,0,0]';
for k=1:1:200   %ʹ��200��������
    time1(k)=k;   
   uc(k)=sin(2*pi/4*k) + 1/5*sin(2*pi/7*k);   %�µĲ��Ժ���
 %   uc(k)=sin(2*pi/3*k*ts) + 1/3*sin(2*pi/6*k*ts);
    yc(k) = u_1+ y_1 * (2*y_2 + 1) / (1+ y_1^2 + y_2^2);  %������ϵͳ
    
    for  j=1:1:9   
         I(j)=x'*w1(:,j);   
  %        Iout(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));
         Iout(j)=1/(1+exp(-I(j))); %���������������ʹ��sigmod����
    end   
    ync(k)=w2'*Iout;         % �������������
    ec(k)=yc(k)-ync(k);    % ��ʶ���
    
    %����������
    x(1)=uc(k);
    x(2)=yc(k);
     x(3)=y_1;
     x(4)=y_2;
    
    u_1=uc(k);
    y_2=y_1;
    y_1=yc(k);
end
figure(4);
plot(time1,yc,'r',time1,ync,'b');
xlabel('times');ylabel('y and yn');
figure(5);
plot(time1,ec,'r');
xlabel('times');ylabel('error');