%%
%CD_Dt0 = CD_Coe(1);   CD_alp = CD_Coe(2);   CD_dDte = CD_Coe(3);  CD_dDta = CD_Coe(4);  CD_dDtr = CD_Coe(5);
% CY_Dt0 = CY_Coe(1);   CY_bta = CY_Coe(2);   CY_dDte = CY_Coe(3);  CY_dDta = CY_Coe(4);  CY_dDtr = CY_Coe(5); 
% CL_Dt0 = CL_Coe(1);   CL_alp = CL_Coe(2);   CL_dDte = CL_Coe(3);  CL_dDta = CL_Coe(4);
% Cl_Dt0 = Clw_Coe(1); C_lw_aero = Clw_Coe(2);Cl_dDte = Clw_Coe(3); Cl_dDta = Clw_Coe(4); Cl_dDtr = Clw_Coe(5);
% Cm_Dt0 = Cmw_Coe(1); C_mw_aero = Clw_Coe(2);Cm_dDte = Cmw_Coe(3); Cm_dDta = Cmw_Coe(4); Cm_dDtr = Cmw_Coe(5);  
% Cn_Dt0 = Cnw_Coe(1); C_nw_aero = Clw_Coe(2);Cn_dDte = Cnw_Coe(3); Cn_dDta = Cnw_Coe(4); Cn_dDtr = Cnw_Coe(5); 
function [CD_Coe, CY_Coe, CL_Coe, Clw_Coe, Cmw_Coe, Cnw_Coe] = AeroCofict4afn(Ma,ALPHA,beta,RE,LE,RUD,p,q,r,V)

global b;    %%翼展
global c;    %%平均气动弦长
%% CD
CD_alp =   + 8.717E-02 - 3.307E-02*Ma + 3.179E-03*ALPHA - 1.25E-04*ALPHA*Ma + 5.036E-03*(Ma^2) - 1.1E-03*(ALPHA^2) ...
        + 1.405E-07*((ALPHA*Ma)^2) - 3.658E-04*(Ma^3) + 3.175E-04*(ALPHA^3) + 1.274E-05*(Ma^4) - 2.985E-05*(ALPHA^4) ...
        - 1.705E-07*(Ma^5) + 9.766E-07*(ALPHA^5);

% CD_RE = + 4.5548E-04 + 2.5411E-05*ALPHA -1.1436E-04*Ma + 3.2187E-06*(ALPHA^2) + 3.0140E-06*(Ma^2) ...
%         - 3.6417E-05*RE - 5.3015E-07*ALPHA*Ma*RE + 6.9629E-06*(RE^2) + 2.1026E-12*(((ALPHA*Ma)*RE)^2);  
CD_dDta = - 3.6417E-05 - 5.3015E-07*ALPHA*Ma + 6.9629E-06*2*RE + 2.1026E-12*2*((ALPHA*Ma)^2)*RE;         

% CD_LE = + 4.5548E-04 + 2.5411E-05*ALPHA -1.1436E-04*Ma + 3.2187E-06*(ALPHA^2) + 3.0140E-06*(Ma^2) ...
%         - 3.6417E-05*LE - 5.3015E-07*ALPHA*Ma*LE + 6.9629E-06*(LE^2) + 2.1026E-12*(((ALPHA*Ma)*LE)^2);           
CD_dDte =  - 3.6417E-05 - 5.3015E-07*ALPHA*Ma + 6.9629E-06*2*LE + 2.1026E-12*2*((ALPHA*Ma)^2)*LE; 

% CD_RUD = + 7.5E-04 - 2.29E-05*ALPHA - 9.69E-05*Ma + 8.76E-07*(ALPHA^2) + 2.7E-06*(Ma^2) ...
%          - 1.83E-06*RUD + 9.13E-09*ALPHA*Ma*RUD + 1.97E-06*(RUD^2) - 1.77E-11*((ALPHA*Ma*RUD)^2);          
CD_dDtr = - 1.83E-06 + 9.13E-09*ALPHA*Ma + 1.97E-06*2*RUD - 1.77E-11*2*((ALPHA*Ma)^2)*RUD;

CD_Dt0 = 2*(4.5548E-04 + 2.5411E-05*ALPHA -1.1436E-04*Ma + 3.2187E-06*(ALPHA^2) + 3.0140E-06*(Ma^2)) + ...
         + 7.5E-04 - 2.29E-05*ALPHA - 9.69E-05*Ma + 8.76E-07*(ALPHA^2) + 2.7E-06*(Ma^2);
% C_D = CDa + CD_RE + CD_LE + CD_RUD; %

CD_Coe(1) = CD_Dt0; CD_Coe(2) =  CD_alp;  CD_Coe(3) = CD_dDte;  CD_Coe(4) = CD_dDta;  CD_Coe(5) = CD_dDtr;

%% CY
CY_bta =  -2.9253E-01*Ma + 2.8803E-03*ALPHA - 2.8943E-04*(ALPHA*Ma)+ 5.4822E-02*(Ma^2) + 7.3535E-04*(ALPHA^2) ...
      - 4.6490E-09*((ALPHA*(Ma^2))^2) - 2.0675E-08*(((ALPHA^2)*Ma)^2) + 4.6205E-06*((ALPHA*Ma)^2) + 2.6144E-11*(((ALPHA^2)* (Ma^2))^2) ...
      - 4.3203E-03*(Ma^3) - 3.7405E-04*(ALPHA^3) + 1.5495E-04*(Ma^4) + 2.8183E-05*(ALPHA^4) ...
      - 2.0829E-06*(Ma^5) -5.2083E-07*(ALPHA^5);  
      
% CY_RE = -1.02E-06 - 1.12E-07*ALPHA + 4.48E-07*Ma + 2.82E-09*(ALPHA^2) - 2.36E-08*(Ma^2) ...
%         + 2.27E-07*RE + 4.11E-09*ALPHA*Ma*RE - 5.04E-08*(RE^2) + 4.5E-14*((ALPHA*Ma*RE)^2);      
CY_dDta = + 2.27E-07 + 4.11E-09*ALPHA*Ma - 5.04E-08*2*RE + 4.5E-14*2*((ALPHA*Ma)^2)*RE;
       
% CY_LE = -(-1.02E-06 - 1.12E-07*ALPHA + 4.48E-07*Ma + 2.82E-09*(ALPHA^2) - 2.36E-08*(Ma^2) ...
%         + 2.27E-07*LE + 4.11E-09*ALPHA*Ma*LE - 5.04E-08*(LE^2) + 4.5E-14*(ALPHA*Ma*LE)^2);         
CY_dDte = -( 2.27E-07 + 4.11E-09*ALPHA*Ma - 5.04E-08*2*LE + 4.5E-14*2*((ALPHA*Ma)^2)*LE);
       
% CY_RUD = -1.43E-18 + 4.86E-20*ALPHA + 1.86E-19*Ma + 3.84E-04*RUD - 1.17E-05*ALPHA*RUD - 1.07E-05*Ma*RUD + 2.6E-07*ALPHA*Ma*RUD;
CY_dDtr = 3.84E-04 - 1.17E-05*ALPHA - 1.07E-05*Ma + 2.6E-07*ALPHA*Ma;

CY_Dt0 =  -1.43E-18 + 4.86E-20*ALPHA + 1.86E-19*Ma;
% C_Y = CYb*beta + CY_RE + CY_LE + CY_RUD; 

CY_Coe(1) = CY_Dt0;  CY_Coe(2) = CY_bta*beta;  CY_Coe(3) = CY_dDte; CY_Coe(4) = CY_dDta;  CY_Coe(5) = CY_dDtr; 

%% CL
CL_alp = - 8.19E-02 + 4.7E-02*Ma + 1.86E-02*ALPHA - 4.73E-04*ALPHA*Ma - 9.19E-03*(Ma^2) - 1.52E-04*(ALPHA^2) + 5.99E-07*((ALPHA*Ma)^2) ...
      + 7.74E-04*(Ma^3) + 4.08E-06*(ALPHA^3) - 2.93E-05*(Ma^4) - 3.91E-07*(ALPHA^4) + 4.12E-07*(Ma^5) + 1.3E-08*(ALPHA^5);

% CL_RE = - 1.45E-05 + 1.01E-04*ALPHA + 7.1E-06*Ma + 4.7E-06*ALPHA*Ma ...
%         - 4.14E-04*RE - 3.51E-06*ALPHA*RE + 8.72E-06*Ma*RE - 1.7E-07*ALPHA*Ma*RE;
CL_dDta = - 4.14E-04 - 3.51E-06*ALPHA + 8.72E-06*Ma - 1.7E-07*ALPHA*Ma;

% CL_LE = - 1.45E-05 + 1.01E-04*ALPHA + 7.1E-06*Ma + 4.7E-06*ALPHA*Ma ...
%         - 4.14E-04*LE - 3.51E-06*ALPHA*LE + 8.72E-06*Ma*LE - 1.7E-07*ALPHA*Ma*LE;
CL_dDte = - 4.14E-04 - 3.51E-06*ALPHA + 8.72E-06*Ma - 1.7E-07*ALPHA*Ma;   

CL_Dt0 = 2*(-1.45E-05 + 1.01E-04*ALPHA + 7.1E-06*Ma + 4.7E-06*ALPHA*Ma);
% C_L = CLa + CL_RE + CL_LE;

CL_Coe(1) = CL_Dt0;  CL_Coe(2) = CL_alp;  CL_Coe(3) = CL_dDte;  CL_Coe(4) = CL_dDta;

%% Cl
Clb = - 1.402E-01 + 3.326E-02*Ma - 7.590E-04*ALPHA + 8.596E-06*(ALPHA*Ma) ...
      - 3.794E-03*(Ma^2) + 2.354E-06*(ALPHA^2) -1.044E-08*((ALPHA*Ma)^2) + 2.219E-04*(Ma^3) - 8.964E-18*(ALPHA^3) ...
      - 6.462E-06*(Ma^4) + 3.803E-19*(ALPHA^4) + 7.419E-08*(Ma^5) -3.353E-21*(ALPHA^5);
  
% Cl_RE = + 3.570E-04 - 9.569E-05*ALPHA - 3.598E-05*Ma + 4.950E-06*(ALPHA^2) + 1.411E-06*(Ma^2) ...
%         + 1.170E-04*RE + 2.794E-08*ALPHA*Ma*RE - 1.160E-06*(RE^2) - 4.641E-11*((ALPHA*Ma*RE)^2);          
Cl_dDta = 1.170E-04 + 2.794E-08*ALPHA*Ma - 1.160E-06*2*RE - 4.641E-11*2*((ALPHA*Ma)^2)*RE;
         
% Cl_LE = -( 3.570E-04 - 9.569E-05*ALPHA - 3.598E-05*Ma + 4.950E-06*(ALPHA^2) + 1.411E-06*(Ma^2) ...
%         + 1.170E-04*LE + 2.794E-08*ALPHA*Ma*LE - 1.160E-06*(LE^2) - 4.641E-11*((ALPHA*Ma*LE)^2));       
Cl_dDte = - (1.170E-04 + 2.794E-08*ALPHA*Ma - 1.160E-06*2*LE - 4.641E-11*2*((ALPHA*Ma)^2)*LE);

% Cl_RUD = - 5.0103E-19 + 6.2723E-20*ALPHA + 2.3418E-20*Ma - 3.4201E-21*(ALPHA*Ma) ... 
%          + 1.1441E-04*RUD - 2.6824E-06*(ALPHA*RUD) - 3.5496E-06*(Ma*RUD) + 5.5547E-08*(ALPHA*Ma)*RUD ;
Cl_dDtr = 1.1441E-04 - 2.6824E-06*ALPHA - 3.5496E-06*Ma + 5.5547E-08*(ALPHA*Ma) ;          

Clp = - 2.99E-01 + 7.47E-02*Ma + 1.38E-03*ALPHA - 8.78E-05*(ALPHA*Ma) - 9.13E-03*(Ma^2) - 2.04E-04*(ALPHA^2) - 1.52E-07*((ALPHA*Ma)^2) ...
      + 5.73E-04*(Ma^3) - 3.86E-05*(ALPHA^3) - 1.79E-05*(Ma^4) + 4.21E-06*(ALPHA^4) + 2.20E-07*(Ma^5) - 1.15E-07*(ALPHA^5);
      
Clr = + 3.82E-01 - 1.06E-01*Ma + 1.94E-03* ALPHA - 8.15E-05*(ALPHA*Ma) + 1.45E-02*(Ma^2) - 9.76E-06*(ALPHA^2) + 4.49E-08*((ALPHA*Ma)^2) ...
      - 1.02E-03*(Ma^3) - 2.70E-07*(ALPHA^3) + 3.56E-05*(Ma^4) + 3.19E-08*(ALPHA^4) - 4.81E-07*(Ma^5) -1.06E-09*(ALPHA^5);

Cl_Dt0 = - 5.0103E-19 + 6.2723E-20*ALPHA + 2.3418E-20*Ma - 3.4201E-21*(ALPHA*Ma);

% Clw = Clb*beta + Cl_RE + Cl_LE + Cl_RUD + Clp*(p*b/V/2) + Clr*(r*b/V/2);%该式里的角度是弧度值
C_lw_aero = Clb*beta + Clp*(p*b/V/2) + Clr*(r*b/V/2);%该式里的角度是弧度值

Clw_Coe(1) = Cl_Dt0; Clw_Coe(2) = C_lw_aero;  Clw_Coe(3) = Cl_dDte; Clw_Coe(4) = Cl_dDta; Clw_Coe(5) = Cl_dDtr; 

%% Cm
Cma = - 2.192E-02 + 7.739E-03*Ma - 2.260E-03*ALPHA + 1.808E-04*(ALPHA*Ma) - 8.849E-04*(Ma^2) + 2.616E-04*(ALPHA^2) ...
      - 2.880E-07*((ALPHA*Ma)^2) + 4.617E-05*(Ma^3) -7.887E-05*(ALPHA^3) - 1.143E-06*(Ma^4) ...
      + 8.288E-06*(ALPHA^4) + 1.082E-08*(Ma^5) - 2.789E-07*(ALPHA^5);

% Cm_RE = - 5.67E-05 - 6.59E-05*ALPHA - 1.51E-06*Ma - 4.46E-06*ALPHA*Ma ...
%         + 2.89E-04*RE + 4.48E-06*ALPHA*RE - 5.87E-06*Ma*RE + 9.72E-08*ALPHA*Ma*RE;
Cm_dDta = 2.89E-04 + 4.48E-06*ALPHA - 5.87E-06*Ma + 9.72E-08*ALPHA*Ma;

% Cm_LE = - 5.67E-05 - 6.59E-05*ALPHA - 1.51E-06*Ma - 4.46E-06*ALPHA*Ma ...
%         + 2.89E-04*LE + 4.48E-06*ALPHA*LE - 5.87E-06*Ma*LE + 9.72E-08*ALPHA*Ma*LE;   
Cm_dDte = 2.89E-04 + 4.48E-06*ALPHA - 5.87E-06*Ma + 9.72E-08*ALPHA*Ma;

% Cm_RUD = - 2.79E-05*ALPHA - 5.89E-08*(ALPHA^2) + 1.58E-03*(Ma^2) + 6.42E-08*(ALPHA^3) - 6.69E-04*(Ma^3) ...
%          - 2.1E-08*(ALPHA^4) + 1.05E-04*(Ma^4) + 3.14E-09*(ALPHA^5) - 7.74E-06*(Ma^5) ...
%          - 2.18E-10*(ALPHA^6) + 2.7E-07*(Ma^6) + 5.74E-12*(ALPHA^7) - 3.58E-09*(Ma^7) ...
%          + 1.43E-07*(RUD^4) - 4.77E-22*(RUD^5) - 3.38E-10*(RUD^6) + 2.63E-24*(RUD^7);        
Cm_dDtr = 1.43E-07*4*(RUD^3) - 4.77E-22*5*(RUD^4) - 3.38E-10*6*(RUD^5) + 2.63E-24*7*(RUD^6);
          
Cmq = - 1.36 + 3.86E-01*Ma + 7.85E-04*ALPHA + 1.4E-04*ALPHA*Ma - 5.42E-02*(Ma^2) + 2.36E-03*(ALPHA^2) - 1.95E-06*((ALPHA*Ma)^2) ...
      + 3.8E-03*(Ma^3) - 1.48E-03*(ALPHA^3) - 1.3E-04*(Ma^4) + 1.69E-04*(ALPHA^4) + 1.71E-06*(Ma^5) - 5.93E-06*(ALPHA^5);

Cm_Dt0 = 2*(- 5.67E-05 - 6.59E-05*ALPHA - 1.51E-06*Ma - 4.46E-06*ALPHA*Ma) + ...
         - 2.79E-05*ALPHA - 5.89E-08*(ALPHA^2) + 1.58E-03*(Ma^2) + 6.42E-08*(ALPHA^3) - 6.69E-04*(Ma^3) ...
         - 2.1E-08*(ALPHA^4) + 1.05E-04*(Ma^4) + 3.14E-09*(ALPHA^5) - 7.74E-06*(Ma^5) ...
         - 2.18E-10*(ALPHA^6) + 2.7E-07*(Ma^6) + 5.74E-12*(ALPHA^7) - 3.58E-09*(Ma^7);
% Cmw = Cma + Cm_RE + Cm_LE + Cm_RUD + Cmq*(q*c/V/2);
C_mw_aero = Cma + Cmq*(q*c/V/2);

Cmw_Coe(1) = Cm_Dt0; Cmw_Coe(2) = C_mw_aero; Cmw_Coe(3) = Cm_dDte; Cmw_Coe(4) = Cm_dDta; Cmw_Coe(5) = Cm_dDtr;

%% Cn
Cnb = + 6.998E-04*ALPHA + 5.9115E-02*Ma -7.525E-05*(ALPHA*Ma) + 2.516E-04*((ALPHA).^2) -1.4824E-02*(Ma^2) - 2.1924E-07*((ALPHA*Ma)^2) ...
      - 1.0777E-04*(ALPHA^3) + 1.2692E-03*(Ma^3) + 1.0707E-08 *((ALPHA*Ma)^3) ...
      + 9.4989E-06*(ALPHA^4) - 4.7098E-05*(Ma^4) - 5.5472E-11*((ALPHA*Ma)^4) ...
      - 2.5953E-07*(ALPHA^5) + 6.4284E-07*(Ma^5) + 8.5863E-14*((ALPHA*Ma)^5);  
     
% Cn_RE = + 2.10E-04 + 1.83E-05*ALPHA - 3.56E-05*Ma -6.39E-07*(ALPHA^2) + 8.16E-07*(Ma^2) ...
%         - 1.30E-05*RE - 8.93E-08*(ALPHA*Ma)*RE + 1.97E-06*(RE^2) + 1.41E-11*((ALPHA*Ma*RE)^2) ;  
Cn_dDta = - 1.30E-05 - 8.93E-08*(ALPHA*Ma) + 1.97E-06*2*RE + 1.41E-11*2*((ALPHA*Ma)^2)*RE ;        

% Cn_LE = -( 2.10E-04 + 1.83E-05*ALPHA - 3.56E-05*Ma -6.39E-07*(ALPHA^2) + 8.16E-07*(Ma^2) ...
%         - 1.30E-05*LE - 8.93E-08*(ALPHA*Ma)*LE + 1.97E-06*(LE^2) + 1.41E-11*((ALPHA*Ma*LE)^2)) ;
Cn_dDte = -( -1.30E-05 - 8.93E-08*(ALPHA*Ma) + 1.97E-06*2*LE + 1.41E-11*2*((ALPHA*Ma)^2)*LE) ;        

% Cn_RUD = + 2.85E-18 - 3.59E-19 *ALPHA -1.26E-19*Ma + 1.57E-20*(ALPHA*Ma) ...
%          - 5.28E-04*RUD + 1.39E-05*(ALPHA*RUD) + 1.65E-05*(Ma*RUD) - 3.13E-07*(ALPHA*Ma)*RUD ;        
Cn_dDtr = - 5.28E-04 + 1.39E-05*ALPHA + 1.65E-05*Ma - 3.13E-07*(ALPHA*Ma)*RUD;      

Cnp = + 3.68E-01 - 9.79E-02*Ma + 7.61E-16*ALPHA + 1.24E-02*(Ma^2) - 4.64E-16*(ALPHA^2) - 8.05E-04*(Ma^3) + 1.01E-16*(ALPHA^3) ...
      + 2.57E-05*(Ma^4) - 9.18E-18*(ALPHA^4) - 3.20E-07*(Ma^5) + 2.96E-19*(ALPHA^5);
      
Cnr = - 2.41 + 5.96E-01*Ma - 2.74E-03*ALPHA + 2.09E-04*(ALPHA*Ma) - 7.57E-02*(Ma^2) + 1.15E-03*(ALPHA^2) - 6.53E-08*((ALPHA*Ma)^2) ...
      + 4.90E-03*(Ma^3) - 3.87E-04*(ALPHA^3) - 1.57E-04*(Ma^4) + 3.60E-05*(ALPHA^4) + 1.96E-06*(Ma^5) - 1.18E-06*(ALPHA^5);

Cn_Dt0 = + 2.85E-18 - 3.59E-19 *ALPHA -1.26E-19*Ma + 1.57E-20*(ALPHA*Ma);

% Cnw = Cnb*beta + Cn_RE + Cn_LE + Cn_RUD + Cnp*(p*b/V/2) + Cnr*(r*b/V/2);%该式里的角度是弧度值
C_nw_aero = Cnb*beta + Cnp*(p*b/V/2) + Cnr*(r*b/V/2);%该式里的角度是弧度值

Cnw_Coe(1) = Cn_Dt0; Cnw_Coe(2) = C_nw_aero; Cnw_Coe(3) = Cn_dDte; Cnw_Coe(4) = Cn_dDta; Cnw_Coe(5) = Cn_dDtr; 

end
