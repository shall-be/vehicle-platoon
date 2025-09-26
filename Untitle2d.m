yalmip('clear')
clear all
close all

%% ����
% ��������
n = 5; % ������
T = 100; % �ܹ���ģ��ʱ�����
delta_t = 0.05; % ����ʱ��
d_min = 0.5; % ������С���
v_des = 15; % ��������
v_max = 20; % �����
L = 10; % ���񳤶�
t = T * delta_t; % �ܹ�ģ��ʱ��
K = t / L; % ����һ�Ż�����

% ��·���β���
noLane = 3; % ������
laneWidth = 3.75; % �й����ٹ�·��ȹ淶
road_right = 0;
road_left = road_right + noLane * laneWidth;

% ��������
steer_limit = 0.3; % ת�����ƣ�������
accel_limit = 4; % ���ٶ����ƣ���λ��ÿ���η���
change_steer_limit = 0.2; % ת�Ǳ仯����
change_accel_limit = 0.3; % �Ӽ��ٶ�����

% ��������
l_f = 4.47/2; % ����ǰ�벿��
l_r = 4.47/2; % �����벿��
len = l_f + l_r; % ������
wid = 1.82; % ������

% ������ʼ״̬
state_init = zeros(4, 1, n);
state_fin = zeros(4, 1, n);
state_init(1, :, :) = [5; 15; 45; 35; 15]';
state_init(2, :, :) = [5; 1; 5; 9; 9]';
state_init(4, :, :) = [17; 14; 14; 13; 15]';

%% ����һ��ͨ����ʼ״̬���Ż�����״̬
% ����ʼ״̬ת��Ϊ����״����
x_init = floor(state_init(1, 1, :) / L) + 1;
y_init = floor(state_init(2, 1, :) / laneWidth) + 1;
v_init = state_init(4, 1, :);

% ������߱���
x = intvar(1, n);
y = intvar(1, n);
x0 = intvar(1, 1, n);
y0 = intvar(1, 1, n);
v0 = sdpvar(1, 1, n);

% �ͷ�����
theta1 = 0.5;
theta2 = 1;

constraints_1 = [];
objective_1 = 0;

for i = 1:n
    objective_1 = objective_1 + abs(y(i) - y0(i)) * theta1;
    objective_1 = objective_1 + abs((x(i) - x0(i)) - K * (v0(i) - v_des)) * theta2;
    constraints_1 = [constraints_1, 1 <= x(i) <= n];
    constraints_1 = [constraints_1, 1 <= y(i) <= noLane];
    for j = i + 1:n
        constraints_1 = [constraints_1, (x(i) - x(j))^2 + (y(i) - y(j))^2 >= 1];
    end
end

parameters_in_1 = {x0, y0, v0};
solutions_out_1 = {x, y};
sequence = optimizer(constraints_1, objective_1, sdpsettings('solver','gurobi','verbose',2), ...
                     parameters_in_1, solutions_out_1);
sol = sequence{x_init, y_init, v_init};

% ��ȡ����״̬
state_fin(1, :, :) = sol{1} * L - 0.5 * L + v_des * t;
state_fin(2, :, :) = sol{2} * laneWidth - 0.5 * laneWidth;
state_fin(4, :, :) = v_des;

%% �������ͨ����ʼ״̬������״̬���ɲο��켣
% ���������������
A0 = [t^3, t^4, t^5;
      3 * t^2, 4 * t^3, 5 * t^4;
      6 * t, 12 * t^2, 20 * t^3];
A_inv = inv(A0);
x_parameter = zeros(6, 1, n);
y_parameter = zeros(6, 1, n);

for i = 1:n
    B1 = [state_fin(1, :, i) - state_init(1, :, i) - t * state_init(4, :, i);
          state_fin(4, :, i) - state_init(4, :, i);
          0];

    B2 = [state_fin(2, :, i) - state_init(2, :, i);
          0;
          0];

    x_parameter(1, :, i) = state_init(1, :, i);
    x_parameter(2, :, i) = state_init(4, :, i);
    y_parameter(1, :, i) = state_init(2, :, i);
    x_parameter(4:6, :, i) = (A_inv * B1)';
    y_parameter(4:6, :, i) = (A_inv * B2)';
end

% �켣ֵ
x_value_ref = zeros(n, T);
y_value_ref = zeros(n, T);

for i = 1:n
    for j = 1:T
        for k = 1:6
            x_value_ref(i, j) = x_value_ref(i, j) + x_parameter(k, :, i) * ((j - 1) * delta_t)^(k - 1);
            y_value_ref(i, j) = y_value_ref(i, j) + y_parameter(k, :, i) * ((j - 1) * delta_t)^(k - 1);
        end
    end
end

% % ���Ʋο��켣
% figure;
% hold on;
% colors = lines(n);
% for i = 1:n
%     plot(x_value_ref(i, :), y_value_ref(i, :), 'Color', colors(i, :), ...
%          'Marker', 'o', 'LineStyle', '-', 'MarkerSize', 0.1);
% end
% hold off;
% xlabel('X Position');
% ylabel('Y Position');
% xlim([-10 150]);
% ylim([0 noLane * laneWidth]);
% title('Reference Trajectories');
% legend('Vehicle 1', 'Vehicle 2', 'Vehicle 3', 'Vehicle 4', 'Vehicle 5', ...
%        'Vehicle 6', 'Vehicle 7', 'Vehicle 8', 'Vehicle 9', 'Vehicle 10');
% saveas(gcf, 'C:\Users\asus\Desktop\˶ʿ����\matlab����\�ο��켣.png');

ref = cell(n, T);
for i = 1:n
    for j = 1:T
        ref{i, j} = zeros(4, 1);
    end
    
    ref{i, 1}(1, 1) = state_init(1, :, i);
    ref{i, 1}(2, 1) = state_init(2, :, i);
    ref{i, 1}(4, 1) = state_init(4, :, i);
end

for i = 1:n
    for j = 2:T
        delta_x = (x_value_ref(i, j) - x_value_ref(i, j - 1)) / delta_t;
        delta_y = (y_value_ref(i, j) - y_value_ref(i, j - 1)) / delta_t;
        v = sqrt((delta_x)^2 + (delta_y)^2);
        ref{i, j}(1, 1) = x_value_ref(i, j);
        ref{i, j}(2, 1) = y_value_ref(i, j);
        ref{i, j}(4, 1) = v;
    end
end



%% ������������ģ�ͽ���
% ��kinematic_bicyle_model.m �Լ� rotation_translation.m



%% �����ģ�MPC
N = 5; %MPC horizon

u = cell(n, N);
u_prev = cell(1, n);
beta = cell(n, N + 1);
z = cell(n, N + 1);
r = cell(n, N + 1);
A = cell(n, N + 1);
b = cell(n, N + 1);

%{

lambda = cell(n, n);
mu = cell(n, n);
s = cell(n, n);

%}

for i = 1:n
    for j = 1:N
        u{i, j} = sdpvar(2, 1);
    end
end

for i = 1:n
    u_prev{i} = sdpvar(2, 1);
end

for i = 1:n
    for j = 1:N + 1
        beta{i, j} = sdpvar(1, 1);
        z{i, j} = sdpvar(4, 1);
        r{i, j} = sdpvar(4, 1);
        A{i, j} = sdpvar(4, 2);
        b{i, j} = sdpvar(4, 1);
    end
end



for i = 1:n
    for j = 1:n
        lambda{i, j} = sdpvar(4, N + 1);
        mu{i, j} = sdpvar(4, N + 1);
        s{i, j} = sdpvar(2, N + 1);
    end
end



constraints_2 = [];
objective_2 = 0;

Q = 0.1 * diag([1, 100, 1, 0.1]);
R = 0.1 * diag([1, 1]);

for k = 1:N
    for i = 1:n
%         objective_2 = objective_2 + (u{i, k} - u_prev{i})' * P * (u{i, k} - u_prev{i});
        objective_2 = objective_2 + (z{i, k} - r{i, k})' * Q * (z{i, k} - r{i, k});
        objective_2 = objective_2 + u{i, k}' * R * u{i, k};
        
        beta{i, k} = atan((l_r / (l_f + l_r)) * tan(u{i, k}(2)));
        constraints_2 = [constraints_2, z{i, k + 1}(1) == z{i, k}(1) + ...
                       delta_t * z{i, k}(4) * cos(z{i, k}(3) + beta{i, k})];
        constraints_2 = [constraints_2, z{i, k + 1}(2) == z{i, k}(2) + ...
                       delta_t * z{i, k}(4) * sin(z{i, k}(3) + beta{i, k})];
        constraints_2 = [constraints_2, z{i, k + 1}(3) == z{i, k}(3) + ...
                       delta_t * (z{i, k}(4) / l_r) * sin(beta{i, k})];
        constraints_2 = [constraints_2, z{i, k + 1}(4) == z{i, k}(4) + delta_t * u{i, k}(1)];
        constraints_2 = [constraints_2, z{i, k}(2) + wid / 2 <= road_left];
        constraints_2 = [constraints_2, z{i, k}(2) - wid / 2 >= road_right];
        constraints_2 = [constraints_2, 0.0 <= z{i, k}(4) <= v_max];
        constraints_2 = [constraints_2, -accel_limit <= u{i, k}(1) <= accel_limit];
        constraints_2 = [constraints_2, -steer_limit <= u{i, k}(2) <= steer_limit];

        if k ~= 1
            constraints_2 = [constraints_2, -change_accel_limit <= u{i, k}(1) - u{i, k - 1}(1) <= change_accel_limit];
            constraints_2 = [constraints_2, -change_steer_limit <= u{i, k}(2) - u{i, k - 1}(2) <= change_steer_limit];
        end

        if k == 1
            constraints_2 = [constraints_2, -change_accel_limit <= u{i, k}(1) - u_prev{i}(1) <= change_accel_limit];
            constraints_2 = [constraints_2, -change_steer_limit <= u{i, k}(2) - u_prev{i}(2) <= change_steer_limit];
        end
        
        
        
        [A{i, k+1}, b{i, k+1}] = rotation_translation([z{i, k+1}(1); z{i, k+1}(2)], z{i, k+1}(3), len, wid);
        
        for j = i + 1:n
            constraints_2 = [constraints_2, b{i, k}' * lambda{i, j}(:, k) + b{j, k}'* mu{i, j}(:, k) <= -d_min];
            constraints_2 = [constraints_2, A{i, k}' * lambda{i, j}(:, k) + s{i, j}(:, k) == 0];
            constraints_2 = [constraints_2, A{j, k}' * mu{i, j}(:, k) - s{i, j}(:, k) == 0];
            constraints_2 = [constraints_2, lambda{i, j}(:, k) >= zeros(4, 1)];
            constraints_2 = [constraints_2, mu{i, j}(:, k) >= zeros(4, 1)];
            constraints_2 = [constraints_2, s{i, j}(1, k)^2 + s{i, j}(2, k)^2 <= 1];
        end
        
        
        
    end
end

for i = 1:n
    objective_2 = objective_2 + (z{i, N + 1} - r{i, N + 1})' * Q * (z{i, N + 1} - r{i, N + 1});
    
    constraints_2 = [constraints_2, z{i, N + 1}(2) + wid / 2 <= road_left];
    constraints_2 = [constraints_2, z{i, N + 1}(2) - wid / 2 >= road_right];
    constraints_2 = [constraints_2, 0.0 <= z{i, N + 1}(4) <= v_max];
    
    
    
    for j = i + 1:n
        constraints_2 = [constraints_2, b{i, N + 1}' * lambda{i, j}(:, N + 1) + ...
                       b{j, N + 1}'* mu{i, j}(:, N + 1) <= -d_min];
        constraints_2 = [constraints_2, A{i, N + 1}' * lambda{i, j}(:, N + 1) + s{i, j}(:, N + 1) == 0];
        constraints_2 = [constraints_2, A{j, N + 1}' * mu{i, j}(:, N + 1) - s{i, j}(:, N + 1) == 0];
        constraints_2 = [constraints_2, lambda{i, j}(:, N + 1) >= zeros(4, 1)];
        constraints_2 = [constraints_2, mu{i, j}(:, N + 1) >= zeros(4, 1)];
        constraints_2 = [constraints_2, s{i, j}(1, N + 1)^2 + s{i, j}(2, N + 1)^2 <= 1];
    end
    
    
    
end

tmp = 1;

parameters_in_2 = {z{:, 1}, r{:}, A{:, 1}, b{:, 1}, u_prev{:}};

solutions_out_2 = {[u{:}], [z{:}]};

%solutions_out_2 = {[u{:}], [z{:}], [lambda{:}], [mu{:}], [s{:}]};

tmp = 2;

% MPC������
options = sdpsettings('solver', 'fmincon', 'verbose', 2);
controller = optimizer(constraints_2, objective_2, options, parameters_in_2, solutions_out_2);

tmp = 3;

U = zeros(2, n);

u_real = cell(n, T);
z_real = cell(n, T);

for i = 1:n
    for j = 1:T
        u_real{i, j} = zeros(2, 1);
        z_real{i, j} = zeros(4, 1);
    end
    z_real{i, 1} = ref{i, 1};
end

% ref_matrix = zeros(4, 6, 10);
% ref_cell = cell(1, n);

for k = 1:T - N - 1 % �ȼ�һ��������Բ���
%     if k > 1
%         break
%     end
        
    for i = 1:n
        [A_0{i, 1}, b_0{i, 1}] = rotation_translation([z_real{i, k}(1); z_real{i, k}(2)], z_real{i, k}(3), len, wid);
    end
    
    tmp = 10 * k + 1;
    
%     for i = 1:n
%         for j = 1:6
%             ref_matrix(:, j, i) = ref{i, j + k - 1};
%         end
%         ref_cell{:, i} = ref_matrix(:, :, i);
%     end
    
    if k ~= 1
        inputs = {z_real{:, k}, ref{:, k:k+N}, A_0{:, 1}, b_0{:, 1}, u_real{:, k - 1}};
    end

    if k == 1
        inputs = {z_real{:, k}, ref{:, k:k+N}, A_0{:, 1}, b_0{:, 1}, u_real{:, k}};
    end
    
    tmp = 10 * k + 2;
    
    [solutions, diagnostics, ~, ~, ~, diag] = controller{inputs};
    
    tmp = 10 * k + 3;
    
    U = solutions{1};
    
    if diagnostics ~= 0
        error('The problem is infeasible');
    else
        disp('it is feasible');
    end
    
    tmp = 10 * k + 4;
    
    for i = 1:n
        u_real{i, k} = U(:, i);
        [x1, y1, psi1, v1] = kinematic_bicycle_model(z_real{i, k}(1), z_real{i, k}(2), ...
                                                     z_real{i, k}(3), z_real{i, k}(4), ...
                                                     delta_t, l_f, l_r, u_real{i, k}(1), u_real{i, k}(2));
        z_real{i, k + 1} = [x1, y1, psi1, v1]';
    end
                                          
    tmp = 10 * k + 5;
end

tmp = 4;

x_value_real = zeros(n, T);
y_value_real = zeros(n, T);

for i = 1:n
    for j = 1:T
        x_value_real(i, j) = z_real{i, j}(1);
        y_value_real(i, j) = z_real{i, j}(2);
    end
end
tmp = 5;

% ����MPC���
figure;
hold on;
colors = lines(n);
for i = 1:n
    plot(x_value_real(i, 1:84), y_value_real(i, 1:84), 'Color', colors(i, :), ...
         'Marker', 'o', 'LineStyle', '-', 'MarkerSize', 0.1);
end
hold off;
xlabel('X Position');
ylabel('Y Position');
xlim([-10 150]);
ylim([0 noLane * laneWidth]);
title('MPC result');
legend('Vehicle 1', 'Vehicle 2', 'Vehicle 3', 'Vehicle 4', 'Vehicle 5', ...
       'Vehicle 6', 'Vehicle 7', 'Vehicle 8', 'Vehicle 9', 'Vehicle 10');
saveas(gcf, 'C:\Users\asus\Desktop\˶ʿ����\matlab����\MPC���.png');
tmp = 6;


%% �����壺����