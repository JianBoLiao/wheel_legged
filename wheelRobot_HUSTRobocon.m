%% HUST Robocon Wheel Robot LQR param
% author ： Vulcan
% date ： 2023.11.12
clear
clc;close

%% 设置符号变量
%    腿质量 轮质量 轮半径 机身质量
% syms mp mw r M L Lm l g Iw Ip Im  % 常数数值量，可以使用后续的机械参数替换

syms L % 保留变腿长设计
% L = 0.272666;

% 质量参数
mw = 3; % wheel wieght
mp = (0.124 + 0.28) * 2; %摆杆质量
M = 12; % body weight
g = 9.793383;

%% 尺寸参数
r = 0.05;
Lm = L; % 摆杆重心到驱动轮轴据距离
l = 0.02; % 机体中心离其转轴距离

%% 惯量参数
Im = 1/12 * M * (0.2125^2 + 0.1^2 ); % 机体绕质心转动惯量
Iw = 0.5 * mw * r^2; % 1/2 M * R ^2
Ip = 1/12 * mp * (L+Lm)^2; % 摆杆绕质心转动惯量
syms dot2_x dot2_phi dot2_theta
% 状态变量
syms dot_theta dot_x dot_phi
syms theta x phi
% 控制变量
syms T Tp
%% 设置中间值方程（已经线性化）
N = (mp + M)*dot2_x + (L * (mp + M)+Lm*M) * dot2_theta - M*l*dot2_phi;
Nm = M * dot2_x + M * (L + Lm )*dot2_theta - M * l*dot2_phi;
P = (mp + M) * g;
Pm = M * g;
%% 设置三组动力学方程
eqn1 = dot2_x == (T - N * r)/(mw*r + Iw/r);
eqn2 = dot2_theta == 1/Ip * ((P *L +Pm * Lm)*theta - (N * L + Nm * Lm ) +Tp - T);
eqn3 = dot2_phi == 1/Im * ( Tp + Nm *l + Pm * l * phi);
%% 求解并化简得到二阶量的线性表达式
temp = solve(eqn1, eqn2, eqn3, dot2_theta, dot2_x, dot2_phi);
temp.dot2_theta = collect(temp.dot2_theta, [theta, phi, T, Tp]);
temp.dot2_x = collect(temp.dot2_x, [theta, phi, T, Tp]);
temp.dot2_phi = collect(temp.dot2_phi, [theta, phi, T, Tp]);
%% 设置状态变量与控制变量并求得对应的子矩阵（一阶状态量3×3，控制量为3×2）
sub_state = [theta; x; phi];
u = [T,Tp];
% matlab 转置矩阵函数：transpose()
[sub_A,uB] = equationsToMatrix([temp.dot2_theta, temp.dot2_x, temp.dot2_phi], transpose(sub_state));

uB = -uB;
[uB,~] = equationsToMatrix(uB(1:size(uB,1)) , u);
%% 得到最终的6×6状态矩阵A与6×2控制矩阵B
% A = zeros(6,6);
% B = zeros(size(A,1),size(u,2));
for i = 1:size(sub_A,1)
    for j = 1:size(sub_A,2)
        A(2*i,2*j-1) = sub_A(i,j);
    end
end
for i = 1:size(sub_A,1)
    A(2*i-1,2*i) = 1;
end

for i = 1:size(uB,1)
    for j = 1:size(uB,2)
        B(2*i,j) = uB(i,j);
    end
end
%% 中间AB矩阵输出
disp(A);
disp(B);
% disp(double(A));
% disp(double(B));

% 计算A矩阵特征值
% [~,D_A] = eig(A);
% D_A

% %% LQR 权重参数设置
% Q = zeros(6,6);
% ratio = 50;
% Q(1, 1) = 0.05 * ratio;
% Q(2, 2) = 0.05 * ratio;
% Q(3, 3) = 1 * ratio;
% Q(4, 4) = 2 * ratio;
% Q(5, 5) = 1 * ratio;
% Q(6, 6) = 1 * ratio;
% 
% R = [20  ,0  ;
%     0   , 5  ];
% 
% %% 变腿长拟合器值域设置
% L_max = 0.35/2;
% L_min = 0.20/2;
% L_inter = linspace(L_min,L_max,100);
% % 变腿长K矩阵计算
% A_temp = zeros(6,6);
% B_temp = zeros(size(A,1),size(u,2));
% K_all_inter = zeros(2,6,size(L_inter,2));
% for i = 1:size(L_inter,2)
%     A_temp = eval(subs(A,{L},{L_inter(i)}));
%     B_temp = eval(subs(B,{L},{L_inter(i)}));
%     K_all_inter(:,:,i) = lqr(A_temp,B_temp,Q,R);
% end
% % 设置拟合阶数
% order = 3;
% K_params = zeros(size(K_all_inter,1) , size(K_all_inter,2),order+1);
% for i = 1:2
%     for j = 1:6
%         K_params(i,j,:) = polyfit(L_inter, K_all_inter(i,j,:), order);
%     end
% end
% %% 拟合效果检验
% kx = 1;
% ky = 1;
% % 真实值                                                    
% plot(L_inter(:),reshape(K_all_inter(kx,ky,:),[1,size(L_inter,2)]),'Color',"r",'LineWidth',1.5);
% hold on;
% % 拟合值
% K_predic = zeros(1,size(L_inter,2));
% for i = 1 : size(L_inter,2)
%     for j = 0:order
%         K_predic(i) = K_predic(i)+ L_inter(i)^(order-j)*K_params(kx,ky,j+1);
%     end
% end
% plot(L_inter(:),K_predic(:),'Color',"b",'LineWidth',1.5)
% 
% %% 拟合参数矩阵打印输出
% fprintf("float K_params[2][6][4] = \n");
% printArrayInCppFormat(K_params);
% 
% function printArrayInCppFormat(A)
% fprintf("{\n");
% for i = 1:size(A, 1)
%     fprintf("\t");
%     fprintf("{\n");
%     for j = 1:size(A, 2)
%         fprintf("\t\t");
%         fprintf("{");
%         for k = 1:size(A, 3)
%             fprintf("%f", A(i, j, k));
%             if k ~= size(A, 3)
%                 fprintf(", ");
%             end
%         end
%         fprintf("}");
%         if j ~= size(A, 2)
%             fprintf(", ");
%         end
%         fprintf("\n");
%     end
%     fprintf("\t}");
%     if i ~= size(A, 1)
%         fprintf(", ");
%     end
%     fprintf("\n");
% end
% fprintf("}\n");
% end









