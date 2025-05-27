% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: This code evaluates the accuracy of the transition matrices 
%              by comparing the Simulink results with those calculated from the matrices.
%              The external circuit used here differs from the one employed to generate the transition matrices.

% [x_now, y_now]=T*[x_his, u_now]

clear all;
clc;

%% load transition matrices and parameters
load ('T_Matrix.mat');% T_tot,Trigger_value, num, idx_output_vi
Ts=1e-6; % Simulation time step, the same as that used in generation
T_end=0.1; % It can be any value

n_trig=num.n_trig;
n_output=num.n_output;
n_state=num.n_state;
n_input=num.n_input;
n_Sim_input=num.n_Sim_input;%% BE method: [x_his, u_now]
n_Sim_output=num.n_Sim_output;%% BE method: [x_now, y_now]

%% Import Simulink simulation results as benchmark
sim('.\Test_T_Matrix_Boost.slx');
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Simulink_Data=[Data_Trigger,Data_State(:,2:end),Data_Input(:,2:end),Data_Output(:,2:end)];
n_tot=size(Simulink_Data,1);
t_tot=Simulink_Data(:, 1);
trig_now_tot=Simulink_Data(:, 2);
Iin_tot=Simulink_Data(:, 3);
Vout_tot=Simulink_Data(:, 4);
Vin_tot=Simulink_Data(:,5);
Iout_tot=Simulink_Data(:, 6);
VMosfet_tot=Simulink_Data(:, 7);
IDiode_tot=Simulink_Data(:, 8);
P_tot=-Vout_tot.*Iout_tot;
trig_now_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Simulink using the previous trigger

%% EMT solver with obtained transition matrices
Iin_pre=zeros(n_tot,1);
Vout_pre=zeros(n_tot,1);
Vin_pre=zeros(n_tot,1);
Iout_pre=zeros(n_tot,1);
VMosfet_pre=zeros(n_tot,1);
IDiode_pre=zeros(n_tot,1);
P_pre=zeros(n_tot,1);
% Source calculations
V1source=zeros(n_tot,1);
V2source=zeros(n_tot,1);
for i=1:n_tot    
    V1source(i)=12;
    V2source(i)=0;
end
Rs1=0.001;
Rs2=20;
Hex=[1/Rs1,0;0,Rs2];

Sim_input=zeros(n_Sim_input,1);
Sim_output=zeros(n_Sim_output,1);
trigger_value_now=zeros(n_tot,1);

% Main simulation loop
for i=1:n_tot
    trigger_value_now(i) = bin2dec(num2str(trig_now_tot(i, :), '%d'));
    idx_T=find(Trigger_value==trigger_value_now(i));
    T=T_tot(:,:,idx_T);

    Sim_input(1:n_state)=Sim_output(1:n_state); % Update x_his
    
    % Coupled EMT solver for calculating u_now
    Tee=T(idx_output_vi,n_state+1:end);
    This=T(idx_output_vi,1:n_state);    
    H_eq=Tee+Hex;
    VIeq=[V1source(i)/Rs1;V2source(i)]-This*Sim_input(1:n_state);
    VIcal=inv(H_eq)*VIeq;        
    VIcal(1)= max(0, VIcal(1)); % VC>0

    Sim_input((n_state+1):n_Sim_input)=VIcal;% Update u_now    
    
    % Calculte using T matrix
    Sim_output=T*Sim_input; 
    
    Sim_output(1)= max(0,Sim_output(1)); % IL>0
    Sim_output(2)= max(0,Sim_output(2)); % VC>0
    
    Vin_pre(i)=VIcal(1);
    Iout_pre(i)=VIcal(2);    
    Iin_pre(i)=Sim_output(1);
    Vout_pre(i)=Sim_output(2);
    VMosfet_pre(i)=Sim_output(3);
    IDiode_pre(i)=Sim_output(4);

    P_pre(i)=-Vout_pre(i)*Iout_pre(i);
end

%% Compare Simulink and computed results
% figure(1), plot(t_tot, Vin_tot, 'r', t_tot, Vin_pre, 'b--');title('Vin'); % Compare Vin
% figure(2), plot(t_tot, Iin_tot, 'r', t_tot, Iin_pre, 'b--'); title('Iin');% Compare Iin
% figure(3), plot(t_tot, Iout_tot, 'r', t_tot,  Iout_pre, 'b--'); title('Iout'); % Compare Iout
% figure(4), plot(t_tot, VMosfet_tot, 'r', t_tot,  VMosfet_pre, 'b--'); title('VMosfet'); % Compare Vswitch
% figure(5), plot(t_tot, IDiode_tot, 'r', t_tot,  IDiode_pre, 'b--');title('IDiode'); 
figure(6), plot(t_tot, Vout_tot, 'r', t_tot, Vout_pre, 'b--'); title('Vout');% Compare Vout
% figure(7), plot(t_tot, P_tot, 'r', t_tot, P_pre, 'b--'); title('P');% Compare P


