% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: Generate transition matrices for all trigger signal combinations 
%              Compares Simulink results with calculated results using calculated transition matrices.

% [x_now, y_now]=T*[x_his, u_his, u_now]

clear all;
clc;

overall_tic = tic;
%% Parameters of Offline Simulation Model
Freq=5e3;% Minimum frequency of the equivalented system
Ts=1e-6;% Simulation time step

% Indices of Variables in the Total Simulink Data
idx_t=1;
idx_trig=2;
idx_state=3:4;
idx_input=5:6;
idx_output=7:8;
idx_output_vi=[1,2];

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

%% Select data from the Simulink data
tic; 
sim('.\Generate_T_Matrix_Boost.slx');
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Simulink_Data=[Data_Trigger,Data_State(:,2:end),Data_Input(:,2:end),Data_Output(:,2:end)];
Data_select=Simulink_Data(1:n_select,:);
t_select = Data_select(:,idx_t);
trig_now_select=Data_select(:,idx_trig);
x_select=Data_select(:, idx_state);
u_select=Data_select(:, idx_input);
y_select=Data_select(:, idx_output);

time1=toc;

%% Classification of trigger signal combinations
% TR method, hence historical triggers are needed
tic;
trig_now_select = [trig_now_select(1,:); trig_now_select(1:n_select-1,:)]; % In simulink，Tn are generated after EMTP
trig_his_select = [trig_now_select(1,:); trig_now_select(1:n_select-1,:)]; % Historical trigger
trig_combine_select=[trig_his_select,trig_now_select];
trig_dec = arrayfun(@(row) bin2dec(num2str(trig_combine_select(row, :), '%d')), 1:n_select)'; % Convert binary to decimal

[Trigger_value, ~, indices] = unique(trig_dec);% Find all unique values
n_Combination = numel(Trigger_value);% Count the number of unique values
fprintf('There are %d control signal combinations\n', n_Combination);
% Find indices for each unique value
n_Combination_point=zeros(n_Combination, 1);% Selected point of every Combination
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

% Calculate the transition matrices for all trigger signal combinations
T_tot = zeros(n_Sim_output,n_Sim_input,n_Combination); 
for i = 1:n_Combination
    idx_mid=idx_matrix(:,i);
    idx_mid=idx_mid(idx_mid ~= 0);

    % Prepare input and output data for least squares minimization
    Input_Data = [x_select(idx_mid-1, :),u_select(idx_mid-1, :),u_select(idx_mid, :)];
    Output_Data =[x_select(idx_mid, :),y_select(idx_mid, :)];
    T=Input_Data \ Output_Data; 
    T_tot(:,:,i) = T';% T' * input(t,:)' =output(t,:)'
    fprintf('Transition matrix  for Combination %s is \n',dec2bin(Trigger_value(i),2*n_trig));
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
save('T_Matrix.mat', 'T_tot','Trigger_value','num','idx_output_vi');

overall_time = toc(overall_tic);
fprintf('The time consumption for the data collection process based on offline simulation is: %.2f s\n', time1);
fprintf('The time consumption for the T-matrix generation process is: %.2f s\n', time2);
fprintf('The time consumption for the total process of data-driven modeling is: %.2f s\n', overall_time);

%% Import Simulink simulation results as benchmark
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
trig_his_tot = [trig_now_tot(1,:); trig_now_tot(1:n_tot-1,:)]; % Historical trigger
trig_combine_tot=[trig_his_tot,trig_now_tot];

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
    V1source(i)=min(t_tot(i)*(2/0.1),2)+min(t_tot(i)*(0.5*sqrt(2)/0.1),0.5*sqrt(2))*...
        sin(2*pi*50*t_tot(i));
    V2source(i)=min(t_tot(i)*(1/0.05),1)+min(t_tot(i)*(0.5*sqrt(2)/0.05),0.5*sqrt(2))*...
        sin(2*pi*50*t_tot(i)+30/180*pi);
end
Rs1=0.01;
Rs2=1;
Hex=[1/Rs1,0;0,Rs2];

Sim_input=zeros(n_Sim_input,1);
Sim_output=zeros(n_Sim_output,1);
trigger_value_now=zeros(n_tot,1);

% Main simulation loop
for i=1:n_tot
    trigger_value_now(i) = bin2dec(num2str(trig_combine_tot(i, :), '%d'));
    idx_T=find(Trigger_value==trigger_value_now(i));
    T=T_tot(:,:,idx_T);

    Sim_input(1:n_state)=Sim_output(1:n_state);% Update x_his
    Sim_input((n_state+1):(n_state+n_input))=Sim_input((n_state+n_input+1):n_Sim_input);% Update u_his
    
    % Coupled EMT solver for calculating u_now
    Tee=T(idx_output_vi,n_state+n_input+1:n_state+n_input+2);
    This=T(idx_output_vi,1:(n_state+n_input));    
    H_eq=Tee+Hex;
    VIeq=[V1source(i)/Rs1;V2source(i)]-This*Sim_input(1:n_state+n_input);
    VIcal=inv(H_eq)*VIeq;      
    VIcal(1)= max(0, VIcal(1)); % VC>0
    Sim_input((n_state+n_input+1):n_Sim_input)=VIcal;% Update u_now  
      
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

