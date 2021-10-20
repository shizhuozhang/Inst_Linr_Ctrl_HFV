clear all;
close all;

Time = zeros(1,1000);
d1 = zeros(1,1000);
d2 = zeros(1,1000);
d3 = zeros(1,1000);
d4 = zeros(1,1000);
d5 = zeros(1,1000);
for i=1:1:1000
    Time(i) = 0.1*i;
    d1 = 0.2*sin(0.5*Time);
    d2 = 0.2*sin(1*Time);  
    d3 = 0.2*sin(1.5*Time);
    d4 = 0.2*sin(2*Time);
end
d5 = d1+ d2+ d3+ d4;
figure(1)
plot(Time,d5,'-r','LineWidth',1.5);
grid on
axis([0 100 -0.8 0.8]);
xlabel('Time (s)','FontSize',13);
ylabel('Disturbance Value ','FontSize',13);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,400]);
set(gca,'FontSize',13);