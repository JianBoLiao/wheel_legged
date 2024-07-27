%% HUST Robocon Wheel Robot LQR param
% author �� Vulcan
% date �� 2023.11.12
clear
clc;close

%% ���÷��ű���
%    ������ ������ �ְ뾶 ��������
% syms mp mw r M L Lm l g Iw Ip Im  % ������ֵ��������ʹ�ú����Ļ�е�����滻

syms L % �������ȳ����
% L = 0.272666;

% ��������
mw = 3; % wheel wieght
mp = (0.124 + 0.28) * 2; %�ڸ�����
M = 12; % body weight
g = 9.793383;

%% �ߴ����
r = 0.05;
Lm = L; % �ڸ����ĵ���������ݾ���
l = 0.02; % ������������ת�����

%% ��������
Im = 1/12 * M * (0.2125^2 + 0.1^2 ); % ����������ת������
Iw = 0.5 * mw * r^2; % 1/2 M * R ^2
Ip = 1/12 * mp * (L+Lm)^2; % �ڸ�������ת������
syms dot2_x dot2_phi dot2_theta
% ״̬����
syms dot_theta dot_x dot_phi
syms theta x phi
% ���Ʊ���
syms T Tp
%% �����м�ֵ���̣��Ѿ����Ի���
N = (mp + M)*dot2_x + (L * (mp + M)+Lm*M) * dot2_theta - M*l*dot2_phi;
Nm = M * dot2_x + M * (L + Lm )*dot2_theta - M * l*dot2_phi;
P = (mp + M) * g;
Pm = M * g;
%% �������鶯��ѧ����
eqn1 = dot2_x == (T - N * r)/(mw*r + Iw/r);
eqn2 = dot2_theta == 1/Ip * ((P *L +Pm * Lm)*theta - (N * L + Nm * Lm ) +Tp - T);
eqn3 = dot2_phi == 1/Im * ( Tp + Nm *l + Pm * l * phi);
%% ��Ⲣ����õ������������Ա��ʽ
temp = solve(eqn1, eqn2, eqn3, dot2_theta, dot2_x, dot2_phi);
temp.dot2_theta = collect(temp.dot2_theta, [theta, phi, T, Tp]);
temp.dot2_x = collect(temp.dot2_x, [theta, phi, T, Tp]);
temp.dot2_phi = collect(temp.dot2_phi, [theta, phi, T, Tp]);
%% ����״̬��������Ʊ�������ö�Ӧ���Ӿ���һ��״̬��3��3��������Ϊ3��2��
sub_state = [theta; x; phi];
u = [T,Tp];
% matlab ת�þ�������transpose()
[sub_A,uB] = equationsToMatrix([temp.dot2_theta, temp.dot2_x, temp.dot2_phi], transpose(sub_state));

uB = -uB;
[uB,~] = equationsToMatrix(uB(1:size(uB,1)) , u);
%% �õ����յ�6��6״̬����A��6��2���ƾ���B
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
%% �м�AB�������
disp(A);
disp(B);
% disp(double(A));
% disp(double(B));

% ����A��������ֵ
% [~,D_A] = eig(A);
% D_A

% %% LQR Ȩ�ز�������
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
% %% ���ȳ������ֵ������
% L_max = 0.35/2;
% L_min = 0.20/2;
% L_inter = linspace(L_min,L_max,100);
% % ���ȳ�K�������
% A_temp = zeros(6,6);
% B_temp = zeros(size(A,1),size(u,2));
% K_all_inter = zeros(2,6,size(L_inter,2));
% for i = 1:size(L_inter,2)
%     A_temp = eval(subs(A,{L},{L_inter(i)}));
%     B_temp = eval(subs(B,{L},{L_inter(i)}));
%     K_all_inter(:,:,i) = lqr(A_temp,B_temp,Q,R);
% end
% % ������Ͻ���
% order = 3;
% K_params = zeros(size(K_all_inter,1) , size(K_all_inter,2),order+1);
% for i = 1:2
%     for j = 1:6
%         K_params(i,j,:) = polyfit(L_inter, K_all_inter(i,j,:), order);
%     end
% end
% %% ���Ч������
% kx = 1;
% ky = 1;
% % ��ʵֵ                                                    
% plot(L_inter(:),reshape(K_all_inter(kx,ky,:),[1,size(L_inter,2)]),'Color',"r",'LineWidth',1.5);
% hold on;
% % ���ֵ
% K_predic = zeros(1,size(L_inter,2));
% for i = 1 : size(L_inter,2)
%     for j = 0:order
%         K_predic(i) = K_predic(i)+ L_inter(i)^(order-j)*K_params(kx,ky,j+1);
%     end
% end
% plot(L_inter(:),K_predic(:),'Color',"b",'LineWidth',1.5)
% 
% %% ��ϲ��������ӡ���
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









