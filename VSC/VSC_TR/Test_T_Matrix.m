% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: This code evaluates the accuracy of the transition matrices 
%              by comparing the Simulink results with those calculated from the generated transition matrices.

% [x_now, y_now]=T*[x_his, u_his, u_now]

clear all;
clc;

load('T_Matrix.mat');
Ts=5e-6;% Simulation time step
T_end=0.2;% Simulation time step

n_trig=num.n_trig;
n_output=num.n_output;
n_state=num.n_state;
n_input=num.n_input;
n_Sim_input=num.n_Sim_input;%% TR method: [x_his, u_his, u_now]
n_Sim_output=num.n_Sim_output;%% TR method: [x_now, y_now]

%% Simulink Data Import 
sim('.\Test_T_Matrix_VSC.slx'); % Run the Simulink model
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Data_Trigger=[Data_Trigger.time,Data_Trigger.signals.values];
Data_Input=[Data_Input.time,Data_Input.signals.values];
Data_Output=[Data_Output.time,Data_Output.signals.values];
Data_State=[Data_State.time,Data_State.signals.values];
Simulink_Data = [Data_Trigger, Data_State(:, 2:end), Data_Input(:, 2:end), Data_Output(:, 2:end)];

n_tot=size(Simulink_Data,1);
t_tot=Simulink_Data(:, 1);
trig_now_tot=Simulink_Data(:, 2:4);
Iabc_tot=Simulink_Data(:, 5:7);
Vabc_tot=Simulink_Data(:, 8:10);
VPWM_tot=Simulink_Data(:, 12:14);
trig_now_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Simulink using the previous trigger
trig_his_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Historical trigger
trig_combine_tot=[trig_his_tot,trig_now_tot];

%% EMT Solver with Obtained Transition Matrices
% External circuit parameters same as Simulink model
Vdc=zeros(n_tot,1);
Va=zeros(n_tot,1);
Vb=zeros(n_tot,1);
Vc=zeros(n_tot,1);
for i=1:n_tot  
    Vdc(i)=1200;
    Va(i)=380*sqrt(2/3)*sin(2*pi*50*t_tot(i));
    Vb(i)=380*sqrt(2/3)*sin(2*pi*50*t_tot(i)-120/180*pi);
    Vc(i)=380*sqrt(2/3)*sin(2*pi*50*t_tot(i)+120/180*pi);
end
Vin=Vdc;
Vex=[Va,Vb,Vc];
Rs=0.01;
Gex=diag([1/Rs,1/Rs,1/Rs]);

% Initialize variables 
Iabc_pre=zeros(n_tot,3);
Vabc_pre=zeros(n_tot,3);
VPWM_pre=zeros(n_tot,3);
Sim_input=zeros(n_Sim_input,1);
Sim_output=zeros(n_Sim_output,1);
trigger_combine_now=zeros(n_tot,1);

% Main simulation loop
flag_fault=0;
flag=zeros(n_tot,1);
mid=zeros(n_tot,1);
I_fault_his=0;
for i=1:n_tot
    trigger_combine_now = bin2dec(num2str(trig_combine_tot(i, :), '%d'));
    idx_T=find(Trigger_value==trigger_combine_now);
    T=T_tot(:,:,idx_T);% Get the corresponding transition matrix

    % Update x_his and u_his
    Sim_input(1:n_state)=Sim_output(1:n_state); % x_his
    Sim_input((n_state+1):(n_state+n_input))=Sim_input((n_state+n_input+1):n_Sim_input); % u_his
  
    % Coupled EMT solver for calculating u_now     
    Tehis=T(idx_yvi_ex,1:(n_state+n_input));
    Tee=T(idx_yvi_ex,n_state+n_input+idx_uvi_ex);  
    Tei=T(idx_yvi_ex,n_state+n_input+idx_uvi_in);
    Vin_now=Vin(i,:)';
    Vex_now=Vex(i,:)'; 

    His_source=Tehis*Sim_input(1:n_state+n_input)+Tei*Vin_now;
    H_eq=Tee+Gex;
    VIeq=Gex*Vex_now-His_source;
    VIcal=inv(H_eq)*VIeq; 
    
    if (t_tot(i) > 0.1) && (t_tot(i) < 0.15)  % Fault occurs
        flag_fault = 1;               
    elseif (t_tot(i) > 0.15)
        I_fault=VIcal(1)/1e-6;
        if(I_fault_his*I_fault<0) % Fault clears
            flag_fault = 0;
        end
        I_fault_his=I_fault;
    end
    if(flag_fault==1)
        VIcal(1)=0;
    end    
    mid(i)=abs(Sim_output(1));
    flag(i)=flag_fault;
    
    Sim_input((n_state+n_input+1):n_Sim_input)=[VIcal;Vin_now]; % update u_now

    % Calculate using T matrix
    Sim_output=T*Sim_input; 

    Vabc_pre(i,:)=VIcal';
    Iabc_pre(i,:)=Sim_output(1:n_state)';
    VPWM_pre(i,:)=Sim_output((n_state+1):n_Sim_output)';
end

%% Compare Simulink and Computed Results
% Define colors for plotting
colors = struct( ...
    'orange',  [1.0, 0.5, 0.05], ...  % Bright orange
    'cyan',    [0.0, 0.6, 1.0],  ...  % Bright cyan
    'blue',    [0.0, 0.45, 0.85], ... % Bright blue
    'gold',    [1.0, 0.8, 0.2],  ...  % Golden yellow
    'magenta', [0.8, 0.2, 0.8],  ...  % Purple-red
    'green',   [0.2, 0.8, 0.2]   ...  % Bright green
);

% Plot PCC Voltage Waveforms
figure(1); hold on;
plot(t_tot, Vabc_tot(:,1), 'Color', colors.orange, 'LineWidth', 1.8); % Va (total)
plot(t_tot, Vabc_pre(:,1), '--', 'Color', colors.cyan, 'LineWidth', 1.8); % Va (predicted)

plot(t_tot, Vabc_tot(:,2), 'Color', colors.blue, 'LineWidth', 1.8); % Vb (total)
plot(t_tot, Vabc_pre(:,2), '--', 'Color', colors.gold, 'LineWidth', 1.8); % Vb (predicted)

plot(t_tot, Vabc_tot(:,3), 'Color', colors.magenta, 'LineWidth', 1.8); % Vc (total)
plot(t_tot, Vabc_pre(:,3), '--', 'Color', colors.green, 'LineWidth', 1.8); % Vc (predicted)

grid on;
legend({'Va (tot)', 'Va (pre)', 'Vb (tot)', 'Vb (pre)', 'Vc (tot)', 'Vc (pre)'}, ...
    'Location', 'northeastoutside');
title('PCC Voltage Waveforms', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Voltage (V)', 'FontSize', 12);
xlim([min(t_tot), max(t_tot)]);
ylim([1.1*min(Vabc_tot(:)), 1.1*max(Vabc_tot(:))]); % Auto-adjust Y-axis range

% Plot PCC Current Waveforms
figure(2); hold on;
plot(t_tot, Iabc_tot(:,1), 'Color', colors.orange, 'LineWidth', 1.8); % Ia (total)
plot(t_tot, Iabc_pre(:,1), '--', 'Color', colors.cyan, 'LineWidth', 0.9); % Ia (predicted)

plot(t_tot, Iabc_tot(:,2), 'Color', colors.blue, 'LineWidth', 1.8); % Ib (total)
plot(t_tot, Iabc_pre(:,2), '--', 'Color', colors.gold, 'LineWidth', 1.8); % Ib (predicted)

plot(t_tot, Iabc_tot(:,3), 'Color', colors.magenta, 'LineWidth', 1.8); % Ic (total)
plot(t_tot, Iabc_pre(:,3), '--', 'Color', colors.green, 'LineWidth', 1.8); % Ic (predicted)

grid on;
legend({'Ia (tot)', 'Ia (pre)', 'Ib (tot)', 'Ib (pre)', 'Ic (tot)', 'Ic (pre)'}, ...
    'Location', 'northeastoutside');
title('PCC Current Waveforms', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Current (A)', 'FontSize', 12);
xlim([min(t_tot), max(t_tot)]);
ylim([1.1*min(Iabc_tot(:)), 1.1*max(Iabc_tot(:))]); % Auto-adjust Y-axis range