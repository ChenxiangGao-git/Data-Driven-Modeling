% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: This code evaluates the accuracy of the transition matrices 
%              by comparing the Simulink results with those calculated from the matrices.

% [x_now, y_now]=T*[x_last, u_last, u_now]

clear all;
clc;

%% load transition matrices and parameters
load ('T_Matrix.mat');% T_tot,Trigger_value, num, idx_output_vi
Ts=1e-6; % Simulation time step, the same as that used in generation
T_end=0.2; % It can be any value

n_trig=num.n_trig;
n_output=num.n_output;
n_state=num.n_state;
n_input=num.n_input;
n_Sim_input=num.n_Sim_input; %% TR method: [x_last, u_last, u_now]
n_Sim_output=num.n_Sim_output;%% TR method: [x_now, y_now]

%% Simulink Data Import
sim('.\Test_T_Matrix_CHB_DAB.slx'); % Run the Simulink model
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Simulink_Data=[Data_Trigger,Data_State(:,2:end),Data_Input(:,2:end),Data_Output(:,2:end)];

n_tot=size(Simulink_Data,1);
t_tot=Simulink_Data(:, 1);
trig_now_tot=Simulink_Data(:, 2:4);
IT1_tot=Simulink_Data(:, 5);
VC1_tot=Simulink_Data(:, 6);
I1_tot=Simulink_Data(:, 7);
V2_tot=Simulink_Data(:, 8);
V1_tot=Simulink_Data(:, 9);
I2_tot=Simulink_Data(:, 10);
VT1_tot=Simulink_Data(:, 11);
VT2_tot=Simulink_Data(:, 12);

% Prepare historical trigger
trig_now_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Simulink using the previous trigger
trig_his_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Historical trigger
trig_combine_tot=[trig_his_tot,trig_now_tot];

%% External circuit
V1source=zeros(n_tot,1);
V2source=zeros(n_tot,1);
for i=1:n_tot 
    V1source(i)=4e3*sin(2*pi*50*t_tot(i));
    V2source(i)=0;
end
R1=0.01;
L1=0.03;
R2=1;
G2=1/R2;
C2=1000e-6;

x1=1+Ts*R1/(2*L1);
x2=1-Ts*R1/(2*L1);
x3=1+Ts*G2/(2*C2);
x4=1-Ts*G2/(2*C2);
T_sub1=[x2/x1,Ts/2/L1/x1,Ts/2/L1/x1];
T_sub2=[x4/x3,Ts/2/C2/x3,Ts/2/C2/x3];

%% Main simulation loop
% initialization
Sim_input=zeros(n_Sim_input,1);
Sim_output=zeros(n_Sim_output,1);
trigger_value_now=zeros(n_tot,1);
s1=zeros(3,1);
s2=zeros(3,1);
VIcal=zeros(2,1);

IT1_pre=zeros(n_tot,1);
VC1_pre=zeros(n_tot,1);
I1_pre=zeros(n_tot,1);
V2_pre=zeros(n_tot,1);
V1_pre=zeros(n_tot,1);
I2_pre=zeros(n_tot,1);
VT1_pre=zeros(n_tot,1);
VT2_pre=zeros(n_tot,1);

% % Coupled EMT solver
for i=1:n_tot
    trigger_value_now(i) = bin2dec(num2str(trig_combine_tot(i, :), '%d'));
    idx_T=find(Trigger_value==trigger_value_now(i));
    T=T_tot(:,:,idx_T);

    Sim_input(1:n_state)=Sim_output(1:n_state);% x_his
    Sim_input((n_state+1):(n_state+n_input))=Sim_input((n_state+n_input+1):n_Sim_input);%u_his

    % Coupled EMT solver for calculating u_now
    Tee=T(n_state+1:n_state+2,n_state+n_input+1:n_state+n_input+2);
    Tehis=T(n_state+1:n_state+2,1:n_state+n_input);   
    His=Tehis*Sim_input(1:(n_state+n_input));

    s1(1)=VIcal(1);
    s1(2)=s1(3);
    s2(1)=VIcal(2);
    s2(2)=s2(3);
    Eq1=(T_sub1(1:2)*s1(1:2)+T_sub1(3)*V1source(i))/T_sub1(3);
    Eq2=(T_sub2(1:2)*s2(1:2)+T_sub2(3)*V2source(i)*G2)/T_sub2(3);
    H_eq=Tee+[1/T_sub1(3),0;0,1/T_sub2(3)];
    VIeq=[Eq1;Eq2]-His;
    VIcal=inv(H_eq)*VIeq;
    VIcal(2)=max(0,VIcal(2));% VC>0
    I1_pre(i)=VIcal(1);
    V2_pre(i)=VIcal(2);

    VIcal2=Tee*VIcal+His;
    V1_pre(i)=VIcal2(1);
    I2_pre(i)=VIcal2(2);
    s1(3)=V1source(i)- V1_pre(i);
    s2(3)=V2source(i)*G2- I2_pre(i); 

    Sim_input((n_state+n_input+1):n_Sim_input)=VIcal;% u_now

    % Calculte using T matrix
    Sim_output=T*Sim_input; 
    Sim_output(2)= max(0,Sim_output(2)); % VC>0

    IT1_pre(i)=Sim_output(1);
    VC1_pre(i)=Sim_output(2);
    V1_pre(i)=Sim_output(3);
    I2_pre(i)=Sim_output(4);
    VT1_pre(i)=Sim_output(5);
    VT2_pre(i)=Sim_output(6);
end


% % % Decoupled EMT solver
% for i=1:n_tot
%     trigger_value_now(i) = bin2dec(num2str(trig_combine_tot(i, :), '%d'));
%     idx_T=find(Trigger_value==trigger_value_now(i));
%     T=T_tot(:,:,idx_T);
% 
%     Sim_input(1:n_state)=Sim_output(1:n_state);% x_his
%     Sim_input((n_state+1):(n_state+n_input))=Sim_input((n_state+n_input+1):n_Sim_input);% u_his
%     Sim_input((n_state+n_input+1):n_Sim_input)=Sim_input((n_state+1):(n_state+n_input));% u_now (latency)
% 
%     % Decoupled EMT solver for calculating u_now
%     Sim_output=T*Sim_input;     
%     Sim_output(2)= max(0,Sim_output(2)); % VC>0
% 
%     IT1_pre(i)=Sim_output(1);
%     VC1_pre(i)=Sim_output(2);
%     V1_pre(i)=Sim_output(3);
%     I2_pre(i)=Sim_output(4);
%     VT1_pre(i)=Sim_output(5);
%     VT2_pre(i)=Sim_output(6);
% 
%     % update of external circuit
%     s1(1)=VIcal(1);
%     s1(2)=s1(3);
%     s1(3)=V1source(i)- V1_pre(i);
%     VIcal(1)=T_sub1*s1;
% 
%     s2(1)=VIcal(2);
%     s2(2)=s2(3);
%     s2(3)=V2source(i)*G2- I2_pre(i);    
%     VIcal(2)=T_sub2*s2;
%     VIcal(2)=max(0,VIcal(2));% VC>0
% 
%     Sim_input((n_state+n_input+1):n_Sim_input)=VIcal;% u_now
%     I1_pre(i)=VIcal(1);
%     V2_pre(i)=VIcal(2);    
% end


%% Compare Simulink and computed results
figure(1), plot(t_tot, IT1_tot, 'r', t_tot, IT1_pre, 'b--'); title('IT1');% Compare IT1
figure(2), plot(t_tot, VC1_tot, 'r', t_tot, VC1_pre, 'b--'); title('VC1');% Compare VC1
figure(3), plot(t_tot, V1_tot, 'r', t_tot, V1_pre, 'b--'); title('V1');% Compare V1
figure(4), plot(t_tot, V2_tot, 'r', t_tot, V2_pre, 'b--'); title('V2');% Compare V2
figure(5), plot(t_tot, I1_tot, 'r', t_tot,  I1_pre, 'b--'); title('I1'); % Compare I1
figure(6), plot(t_tot, I2_tot, 'r', t_tot,  I2_pre, 'b--'); title('I2');% Compare I2
figure(7), plot(t_tot, VT1_tot, 'r', t_tot, VT1_pre, 'b--'); title('VT1');% Compare VT1
figure(8), plot(t_tot, VT2_tot, 'r', t_tot, VT2_pre, 'b--'); title('VT2');% Compare VT2

