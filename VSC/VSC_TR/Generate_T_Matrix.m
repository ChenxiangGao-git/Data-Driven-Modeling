% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: Generate transition matrices for all trigger signal combinations 
%              Compares Simulink results with calculated results using the generated transition matrices.

% [x_now, y_now]=T*[x_his, u_his, u_now]

clear all;
clc;

overall_tic = tic;
%% Parameters of Offline Simulation Model
Freq=50;% Minimum frequency of the equivalented system
Ts=5e-6;% Simulation time step

% Indices of Variables in the Total Simulink Data
idx_t = 1;                          % Time index
idx_trig = 2:4;                     % Trigger signals
idx_state = 5:7;                    % State variables
idx_input = 8:11;                   % Input variables
idx_output = 12:14;                 % Output variables
idx_yvi_ex = 1:3;             % Indices of V or I in [x_now, y_now]
idx_uvi_ex = 1:3;            % Indices of external input in [u]
idx_uvi_in = 4;              % Indices of internal input in [u]

n_trig = numel(idx_trig);
n_state = numel(idx_state);
n_input = numel(idx_input);
n_output = numel(idx_output);

n_Sim_input=n_state+2*n_input;   %% TR method: [x_his, u_his, u_now]
n_Sim_output=n_state+n_output;   %% TR method: [x_now, y_now]

T_end=n_Sim_input/Freq;% Minimum simulation time to guarantee a solution for T for every switch combination
% T_end=0.5; % A larger sampling time can be set to modestly improve accuracy.
n_select=floor(T_end/Ts);
fprintf('The size of T is %d × %d.\n', n_Sim_output, n_Sim_input);
fprintf('The minimum number of selected data points required for each control signal combination is %d.\n', n_Sim_input);
fprintf('The required simulation time for the offline model is %f s.\n', T_end);
fprintf('\n');

%% Simulink Data Import
tic; 
sim('.\Generate_T_Matrix_VSC.slx'); % Run the Simulink model
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Data_Trigger=[Data_Trigger.time,Data_Trigger.signals.values];
Data_Input=[Data_Input.time,Data_Input.signals.values];
Data_Output=[Data_Output.time,Data_Output.signals.values];
Data_State=[Data_State.time,Data_State.signals.values];
Simulink_Data = [Data_Trigger, Data_State(:, 2:end), Data_Input(:, 2:end), Data_Output(:, 2:end)];

Data_select=Simulink_Data(1:n_select,:);
t_select = Data_select(:,idx_t);
trig_now_select=Data_select(:,idx_trig);

time1=toc;

%% Classification of Trigger Combinations
tic;
% TR method, hence historical triggers are needed
trig_now_select = [trig_now_select(1,:); trig_now_select(1:n_select-1,:)]; % In simulink，Tn are generated after EMTP
trig_his_select = [trig_now_select(1,:); trig_now_select(1:n_select-1,:)]; % Historical trigger
trig_combine_select=[trig_his_select,trig_now_select];
trig_dec = arrayfun(@(row) bin2dec(num2str(trig_combine_select(row, :), '%d')), 1:n_select)'; % Convert binary to decimal

[Trigger_value, ~, indices] = unique(trig_dec);% Find unique trigger combinations
n_Combination = numel(Trigger_value);
fprintf('There are %d control signal combinations\n', n_Combination);

n_Combination_point=zeros(n_Combination, 1);% Selected point of every Combination
% Find indices for each unique combination
max_length = max(arrayfun(@(value) numel(find(trig_dec == value)), Trigger_value));
idx_matrix=zeros(max_length, n_Combination);
for i = 1:n_Combination
    value = Trigger_value(i);
    idx_mid = find(trig_dec == value);
    idx_mid(idx_mid==1) = 0;% Remove the first loop as it has no historical items    
    n_Combination_point(i)=length(idx_mid);
    idx_matrix(1:length(idx_mid),i)=idx_mid;
    fprintf('Combination %d: %s (%d), Number of points: %d\n', i, dec2bin(Trigger_value(i),2*n_trig),Trigger_value(i),n_Combination_point(i));
end
fprintf('\n');

% Calculate Transition Matrices for Trigger Combinations
T_tot = zeros(n_Sim_output,n_Sim_input,n_Combination); 
x_select=Data_select(:, idx_state);
u_select=Data_select(:, idx_input);
y_select=Data_select(:, idx_output);

for i = 1:n_Combination
    idx_mid=idx_matrix(:,i);
    idx_mid=idx_mid(idx_mid ~= 0);

    % Prepare input and output data for least squares minimization
    Input_Data = [x_select(idx_mid-1, :),u_select(idx_mid-1, :),u_select(idx_mid, :)];
    Output_Data =[x_select(idx_mid, :),y_select(idx_mid, :)];
    T=Input_Data \ Output_Data; 
    T_tot(:,:,i) = T';% T' * input(t,:)' =output(t,:)'
    fprintf('Transition matrix  for Combination %s is \n',dec2bin(Trigger_value(i),n_trig));
    disp(T');
end

time2=toc;

%% Save Results
num.n_state=n_state;
num.n_input=n_input;
num.n_output=n_output;
num.n_trig=n_trig;
num.n_Combination=n_Combination;
num.n_Sim_input=n_Sim_input;
num.n_Sim_output=n_Sim_output;
save('T_Matrix.mat', 'T_tot','Trigger_value','num','idx_yvi_ex','idx_uvi_ex','idx_uvi_in');

overall_time = toc(overall_tic);
fprintf('The time consumption for the data collection process based on offline simulation is: %.2f s\n', time1);
fprintf('The time consumption for the T-matrix generation process is: %.2f s\n', time2);
fprintf('The time consumption for the total process of data-driven modeling is: %.2f s\n', overall_time);


%% Import Simulink Simulation Results for Comparison
n_tot=size(Simulink_Data,1);
t_tot=Simulink_Data(:, 1);
trig_now_tot=Simulink_Data(:, 2:4);
Iabc_tot=Simulink_Data(:, 5:7);
Vabc_tot=Simulink_Data(:, 8:10);
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
    Vdc(i)=1200+min(t_tot(i)*(10/0.05),10)*sin(2*pi*50*t_tot(i));
    Va(i)=min(t_tot(i)*(300/0.05),300)*sin(2*pi*60*t_tot(i));
    Vb(i)=min(t_tot(i)*(400/0.05),400)*sin(2*pi*100*t_tot(i)+120/180*pi);
    Vc(i)=min(t_tot(i)*(500/0.05),500)*sin(2*pi*50*t_tot(i)+30/180*pi);
end
Vin=Vdc;
Vex=[Va,Vb,Vc];
Gex=diag([1/0.01,1/0.3,1/0.5]);

% Initialize variables 
Iabc_pre=zeros(n_tot,3);
Vabc_pre=zeros(n_tot,3);
Sim_input=zeros(n_Sim_input,1);
Sim_output=zeros(n_Sim_output,1);
trigger_combine_now=zeros(n_tot,1);

% Main simulation loop
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

    Sim_input((n_state+n_input+1):n_Sim_input)=[VIcal;Vin_now]; % update u_now

    % Calculate using T matrix
    Sim_output=T*Sim_input; 

    Vabc_pre(i,:)=VIcal';
    Iabc_pre(i,:)=Sim_output(1:n_state)';
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
plot(t_tot, Iabc_pre(:,1), '--', 'Color', colors.cyan, 'LineWidth', 1.8); % Ia (predicted)

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

