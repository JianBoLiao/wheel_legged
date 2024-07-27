%% 角度加载
rad_AB = zeros([4, size(app.MotorPosition_OUT, 2)]);
rad_ED = zeros([4, size(app.MotorPosition_OUT, 2)]);

if app.Go_2023 == true
    rad_AB(1, :) = 1.5 * pi + app.MotorPosition_OUT(1, :); % AB of Leg1 refer to Motor1_0
    rad_ED(1, :) = 1.5 * pi - app.MotorPosition_OUT(2, :); % AD of Leg1 refer to Motor1_1
    rad_AB(2, :) = 1.5 * pi - app.MotorPosition_OUT(4, :); % AB of Leg2 refer to Motor2_1
    rad_ED(2, :) = 1.5 * pi + app.MotorPosition_OUT(3, :); % AD of Leg2 refer to Motor2_0
    rad_AB(3, :) = 1.5 * pi + app.MotorPosition_OUT(5, :); % AB of Leg1 refer to Motor1_0
    rad_ED(3, :) = 1.5 * pi - app.MotorPosition_OUT(6, :); % AD of Leg1 refer to Motor1_1
    rad_AB(4, :) = 1.5 * pi - app.MotorPosition_OUT(8, :); % AB of Leg2 refer to Motor2_1
    rad_ED(4, :) = 1.5 * pi + app.MotorPosition_OUT(7, :); % AD of Leg2 refer to Motor2_0
end

if app.WheelFeet == true
    rad_AB(1, :) = 1.5 * pi - app.MotorPosition_OUT(1, :);
    rad_ED(1, :) = 1.5 * pi - app.MotorPosition_OUT(2, :);
    rad_AB(2, :) = 1.5 * pi + app.MotorPosition_OUT(3, :);
    rad_ED(2, :) = 1.5 * pi + app.MotorPosition_OUT(4, :);
    rad_AB(3, :) = 1.5 * pi - app.MotorPosition_OUT(5, :);
    rad_ED(3, :) = 1.5 * pi - app.MotorPosition_OUT(6, :);
    rad_AB(4, :) = 1.5 * pi + app.MotorPosition_OUT(7, :);
    rad_ED(4, :) = 1.5 * pi + app.MotorPosition_OUT(8, :);
end

%% 运动学正解模拟
A.x = 0.5 * app.LEA;
A.y = 0;
E.x = -0.5 * app.LEA;
E.y = 0;

for i = 1:4
    h(i) = animatedline(app.AxesHandles(i), 'Color', [111/255, 148/255, 205/255], 'LineStyle', '-', 'LineWidth', 2);
end

for i = 1:size(rad_AB, 2)

    for j = 1:4

        if app.LEA <= 0.001
            %% 按照四足连杆结构计算
            D.x = app.LDE * cos(rad_ED(j, i));
            D.y = app.LDE * sin(rad_ED(j, i));
            B.x = app.LAB * cos(rad_AB(j, i));
            B.y = app.LAB * sin(rad_AB(j, i));
            rad_AC = (rad_AB(j, i) + rad_ED(j, i)) * 0.5;
            Angle_DAB = rad_AB(j, i) - rad_ED(j, i);
            Angle_DAC = 0.5 * Angle_DAB;
            half_BD = app.LDE * sin(Angle_DAC);
            Length_AC = app.LDE * cos(Angle_DAC) + sqrt(app.LCD ^ 2 - half_BD ^ 2);
            C.x = Length_AC * cos (rad_AC);
            C.y = Length_AC * sin (rad_AC);
        else
            %% 按照五连杆轮足结构计算
            rad_AB(j, i) = rad_AB(j, i) - pi;
            rad_ED(j, i) = rad_ED(j, i) - pi;

            B.x = -A.x + app.LAB * cos(rad_AB(j, i));
            B.y = A.y + app.LAB * sin(rad_AB(j, i));
            D.x = -E.x + app.LDE * cos(rad_ED(j, i));
            D.y = E.y + app.LDE * sin(rad_ED(j, i));
            lengthBD = distance_p2p(app, B.x, B.y, D.x, D.y);
            A0 = 2 * app.LBC * (D.x - B.x);
            B0 = 2 * app.LBC * (D.y - B.y);
            C0 = app.LBC ^ 2 + lengthBD ^ 2 - app.LCD ^ 2;
            theta2 = 2 * atan((B0 + sqrt(A0 ^ 2 + B0 ^ 2 - C0 ^ 2)) / (A0 + C0));
            C.x = B.x + app.LBC * cos(theta2);
            C.y = B.y + app.LBC * sin(theta2);

            C.x =- C.x;
            C.y =- C.y;
            B.x =- B.x;
            B.y =- B.y;
            D.x =- D.x;
            D.y =- D.y;

            %                         B.x = A.x + app.LAB * cos(rad_AB(j,i));
            %                         B.y = A.y + app.LAB * sin(rad_AB(j,i));
            %                         D.x = E.x + app.LDE * cos(rad_ED(j,i));
            %                         D.y = E.y + app.LDE * sin(rad_ED(j,i));
            %                         lengthBD = distance_p2p(app,B.x,B.y,D.x,D.y);
            %                         A0 = 2 * app.LBC * (-D.x + B.x);
            %                         B0 = 2 * app.LBC * (-D.y + B.y);
            %                         C0 = app.LBC^2 + lengthBD^2 - app.LCD^2;
            %                         theta2 = 2 * atan( (B0 + sqrt(A0^2+B0^2-C0^2)) / (A0 + C0) );
            %                         C.x = B.x + app.LBC * cos(theta2);
            %                         C.y = B.y - app.LBC * sin(theta2);

        end

        %                     if i == 1
        %                         % 第一次绘图时关闭hold使得坐标区自动刷新
        %                         hold(app.AxesHandles(j),"off");
        %                     else
        hold(app.AxesHandles(j), "on");

        addpoints(h(j), C.x, C.y);
        plot(app.AxesHandles(j), [A.x, B.x, C.x, D.x, E.x, A.x], [A.y, B.y, C.y, D.y, E.y, A.y], 'LineWidth', 0.1);
    end

    pause(0.05);
end

clear("rad_ED","rad_AB");
