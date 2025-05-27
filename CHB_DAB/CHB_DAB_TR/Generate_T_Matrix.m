% ==================================================================================================
% Author: Chenxiang Gao @ Shanghai Jiao Tong University, gaocx_22@sjtu.edu.cn
% ==================================================================================================
% """
% Description: Generate transition matrices for all trigger signal combinations 
%              Compares Simulink results with calculated results using calculated transition matrices.

% [x_now, y_now]=T*[x_last, u_last, u_now]

clear all;
clc;

overall_tic = tic;
%% Parameters of Offline Simulation Model
Freq=50;% Minimum frequency of the equivalented system (for CHB, ac side is 50Hz)
Ts=1e-6;% Simulation time step

% Indices of Variables in the Total Simulink Data
idx_t=1;
idx_trig=2:4;
idx_state=5:6;% IT1,UC1
idx_input=7:8;% I1,V2
idx_output=9:12;% V1,I2, VT1,VT2

n_trig = numel(idx_trig);
n_state = numel(idx_state);
n_input = numel(idx_input);
n_output = numel(idx_output);

n_Sim_input=n_state+2*n_input;   %% TR method: [x_last, u_last, u_now]
n_Sim_output=n_state+n_output;   %% TR method: [x_now, y_now]

T_start=0.01;% To avoid errors caused by overly small data in the initial stages
T_end=T_start+n_Sim_input/Freq;% Minimum simulation time to guarantee a solution for T for every switch combination
% T_end=0.5; % A larger sampling time can be set to modestly improve accuracy.
n_start=floor(T_start/Ts);
n_end=floor(T_end/Ts);
n_select=n_end-n_start+1;
fprintf('The size of T is %d × %d.\n', n_Sim_output, n_Sim_input);
fprintf('The minimum number of selected data points required for each control signal combination is %d.\n', n_Sim_input);
fprintf('The data sampling time interval for the offline model is [%.2f s, %.2f s].\n', T_start, T_end);
fprintf('\n');

%% Simulink Data Import
tic; 
sim('.\Generate_T_Matrix_CHB_DAB.slx');
system(['.\Temp_Del.bat', ' >nul 2>nul']);
Simulink_Data=[Data_Trigger,Data_State(:,2:end),Data_Input(:,2:end),Data_Output(:,2:end)];
Data_select=Simulink_Data(n_start:n_end,:);
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
n_Combination_point=zeros(n_Combination, 1);% Number of selected point of every combination
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

    T = Input_Data \ Output_Data;
    T_tot(:,:,i) = T';% T' * input(t,:)' =output(t,:)'
    fprintf('Transition matrix for Combination %s is \n',dec2bin(Trigger_value(i),2*n_trig));
    disp(T');
end

time2=toc;
%% Output Data
num.n_state=n_state;
num.n_input=n_input;
num.n_output=n_output;
num.n_trig=n_trig;
num.n_Combination=n_Combination;
num.n_Sim_input=n_Sim_input;
num.n_Sim_output=n_Sim_output;
save('T_Matrix.mat', 'T_tot','Trigger_value','num');

overall_time = toc(overall_tic);
fprintf('The time consumption for the data collection process based on offline simulation is: %.2f s\n', time1);
fprintf('The time consumption for the T-matrix generation process is: %.2f s\n', time2);
fprintf('The time consumption for the total process of data-driven modeling is: %.2f s\n', overall_time);

%% Import Simulink simulation results as benchmark
n_tot=size(Simulink_Data,1);
t_tot=Simulink_Data(:, 1);
trig_now_tot=Simulink_Data(:, idx_trig);
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
    V1source(i)=3e3*sin(2*pi*50*t_tot(i));
    V2source(i)=min(t_tot(i)*(5/0.05),5)*sin(2*pi*50*t_tot(i))+...
                min(t_tot(i)*(100/0.05),100);
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



%% Compare Simulink and computed results
figure(1), plot(t_tot, IT1_tot, 'r', t_tot, IT1_pre, 'b--'); title('IT1');% Compare IT1
figure(2), plot(t_tot, VC1_tot, 'r', t_tot, VC1_pre, 'b--'); title('VC1');% Compare VC1
figure(3), plot(t_tot, V1_tot, 'r', t_tot, V1_pre, 'b--'); title('V1');% Compare V1
figure(4), plot(t_tot, V2_tot, 'r', t_tot, V2_pre, 'b--'); title('V2');% Compare V2
figure(5), plot(t_tot, I1_tot, 'r', t_tot,  I1_pre, 'b--'); title('I1'); % Compare I1
figure(6), plot(t_tot, I2_tot, 'r', t_tot,  I2_pre, 'b--'); title('I2');% Compare I2
figure(7), plot(t_tot, VT1_tot, 'r', t_tot, VT1_pre, 'b--'); title('VT1');% Compare VT1
figure(8), plot(t_tot, VT2_tot, 'r', t_tot, VT2_pre, 'b--'); title('VT2');% Compare VT2


