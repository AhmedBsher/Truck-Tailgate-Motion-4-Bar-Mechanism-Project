clc; clear; close all;
format long

% === Step 1: Read CSV File ===
filename = '12.csv';
data = readtable(filename);
read_csv_matlab('theta.csv');
read_csvS_matlab('omega.csv');
read_csvSS_matlab('alpha.csv');

% === Extract Points ===
O_x = data.Anchor_O_X; O_y = data.Anchor_O_Y;
A_x = data.Connector_A_X; A_y = data.Connector_A_Y;
Q_x = data.Anchor_Q_X; Q_y = data.Anchor_Q_Y;
B_x = data.Connector_B_X; B_y = data.Connector_B_Y;

% === Simulation Settings ===
crank_rpm = 3;
omega1_desired = 18; % 3 RPM = 18∞/s

% Calculate time vector based on constant crank speed
theta1 = atan2d(A_y - O_y, A_x - O_x);
time = (theta1 - min(theta1)) / omega1_desired;
dt = mean(diff(time));

% Visualization
figure;
set(gcf, 'Position', [100 100 1200 800]);
axis equal; grid on;
xlabel('X Position (mm)'); ylabel('Y Position (mm)');
title_str = sprintf('4-Bar Mechanism Simulation - Crank Speed: %.1f RPM', crank_rpm);
title(title_str);
hold on;

% Initialize links
links = gobjects(4,1);
links(1) = plot([0 0], [0 0], 'r', 'LineWidth', 2);
links(2) = plot([0 0], [0 0], 'g', 'LineWidth', 2);
links(3) = plot([0 0], [0 0], 'b', 'LineWidth', 2);
links(4) = plot([0 0], [0 0], 'k--', 'LineWidth', 2);
h_points = plot([0 0 0 0], [0 0 0 0], 'ko', 'MarkerFaceColor', 'k');

% Set axis limits
all_x = [O_x; A_x; B_x; Q_x];
all_y = [O_y; A_y; B_y; Q_y];
axis([min(all_x)-10 max(all_x)+10 min(all_y)-10 max(all_y)+10]);

% Animation Loop
for i = 1:length(time)
    % Update positions
    set(links(1), 'XData', [O_x(i) A_x(i)], 'YData', [O_y(i) A_y(i)]);
    set(links(2), 'XData', [A_x(i) B_x(i)], 'YData', [A_y(i) B_y(i)]);
    set(links(3), 'XData', [B_x(i) Q_x(i)], 'YData', [B_y(i) Q_y(i)]);
    set(links(4), 'XData', [Q_x(i) O_x(i)], 'YData', [Q_y(i) O_y(i)]);
    set(h_points, 'XData', [O_x(i) A_x(i) B_x(i) Q_x(i)], 'YData', [O_y(i) A_y(i) B_y(i) Q_y(i)]);
    
    % Update title
    current_title = sprintf('%s | Frame %d/%d', title_str, i, length(time));
    title(current_title);
    
    drawnow;
    pause(8*dt);
end
%% SS
results = table();
row = 1;

%% P REP %%
% ?????? O (?????)
xO = 0;
yO = 0;

% ????? ??????? ???????? (?????? ??????)
theta2_deg = 326.28;

% ??????? ??????????
r2 = 120;  % ??? ??????? OA
r3 = 150;  % ??? ?????? AB
r4 = 100;  % ??? ?????? BQ

% ???? ?????? Q (?????)
xQ = 0;
yQ = 85;

% ------- ??????? ???????? -------
theta2 = deg2rad(theta2_deg);

% ------- 1. ???? ?????? A -------
xA = xO + r2 * cos(theta2);
yA = yO + r2 * sin(theta2);

% ------- 2. ?????? d -------
dx = xQ - xA;
dy = yQ - yA;
d = hypot(dx, dy);  % ??? sqrt(dx^2 + dy^2)
theta_d = atan2(dy, dx);  % ????? ?????? d

% ------- 3. ???? ??????? ? -------
cos_beta = (r3^2 + d^2 - r4^2) / (2 * r3 * d);
cos_beta = min(max(cos_beta, -1), 1);  % ???? ??? ????
beta = acos(cos_beta);  % ?????????
beta_deg = rad2deg(beta);

% ------- 4. ???? ?3 -------
theta3 = theta_d - beta;
theta3_deg = mod(rad2deg(theta3), 360);  % ????? ???????

% ------- 5. ???? ?4 -------
% ???????? ????? ?????? ??????
sin_theta4 = (r3 * sin(theta3) - d * sin(theta_d)) / r4;
cos_theta4 = (r3 * cos(theta3) - d * cos(theta_d)) / r4;
theta4 = atan2(sin_theta4, cos_theta4);
theta4_deg = mod(rad2deg(theta4), 360);

%% V&A REP %%
% ????????%
omega2 = 0.314159; % rad/s
alpha2=0;
% V %
% ?????? 1: ???? ?????? ??? ?????? A
VAx = -omega2 * r2 * sin(theta2);
VAy =  omega2 * r2 * cos(theta2);

% ?????? 2: ???? omega3
omega3 = (VAx * cos(theta4) + VAy * sin(theta4)) / (r3 * sin(theta3 - theta4));

% ?????? 3: ???? omega4
omega4 = (-VAx * sin(theta4) + VAy * cos(theta4) + r3 * omega3 * cos(theta3 - theta4)) / r4;
% A %
a_AX=-omega2^2*r2*cos(theta2)-alpha2*r2*sin(theta2);
a_AY=-omega2^2*r2*sin(theta2)+alpha2*r2*cos(theta2);
alpha3=(omega4^2*r4-omega3^2*r3*cos(theta3-theta4)+a_AX*cos(theta4)+a_AY*sin(theta4))/(r3*sin(theta3-theta4));
alpha4=(omega3^2*r3-omega4^2*r4*cos(theta4-theta3)-a_AX*cos(theta3)-a_AY*sin(theta3))/(r4*sin(theta4-theta3));

%% === Force & Torque Analysis REP ===
% ????? (????)
m2 = 4.13; m3 = 6.2; m4 = 3.42;

% ?????? (??*??^2)
I2 = 8496.82; I3 = 18438.11; I4 = 5089.54;

% ??????
Tt_resist = 3e-5; % N∑m

% ??????? ??? SI
m2 = m2 * 1e-3; m3 = m3 * 1e-3; m4 = m4 * 1e-3;
I2 = I2 * 1e-9; I3 = I3 * 1e-9; I4 = I4 * 1e-9;
r2 = r2 * 1e-3; r3 = r3 * 1e-3; r4 = r4 * 1e-3;

% ??????? ??????
ag2x=-omega2^2*(r2/2)*cos(theta2)-alpha2*(r2/2)*sin(theta2);
ag2y=-omega2^2*(r2/2)*sin(theta2)+alpha2*(r2/2)*cos(theta2);

ag3x=a_AX/1000+(r3/2)*((-omega3^2)*cos(theta3)-alpha3*sin(theta3));
ag3y=a_AY/1000+(r3/2)*((-omega3^2)*sin(theta3)+alpha3*cos(theta3));

ag4x=(r4/2)*(-omega4^2*cos(theta4)-alpha4*sin(theta4));
ag4y=(r4/2)*(-omega4^2*sin(theta4)+alpha4*cos(theta4));

%Inertia Forces
x2=-m2*ag2x;   y2=-m2*ag2y;
x3=-m3*ag3x;   y3=-m3*ag3y;
x4=-m4*ag4x;   y4=-m4*ag4y;
%T
syms X43 Y43;  
eq11 =-y3*(r3/2)*cos(theta3)-Y43*r3*cos(theta3)+x3*(r3/2)*sin(theta3)+X43*r3*sin(theta3);  % ???????? ??????
eq22 =Tt_resist-x4*(r4/2)*sin(theta4)-y4*(r4/2)*cos(theta4)+X43*r4*sin(theta4)+Y43*r4*cos(theta4);  % ???????? ???????
sol = solve([eq11, eq22], [X43, Y43]); 
x43_val = double(sol.X43);
y43_val = double(sol.Y43);
x23=-x43_val-x3;
y23=-y43_val-y3;
M=x2*(r2/2)*sin(theta2)-y2*(r2/2)*cos(theta2)-x23*r2*sin(theta2)+y23*r2*cos(theta2);


%%  P LEP %%
% ????? ??????? ???????? (?????? ??????)
theta22_deg = 64.74;
r2=r2*1000;
r3=r3*1000;
r4=r4*1000;
% ------- ??????? ???????? -------
theta22 = deg2rad(theta22_deg);

% ------- 1. ???? ?????? A -------
xAA = xO + r2 * cos(theta22);
yAA = yO + r2 * sin(theta22);

% ------- 2. ?????? d -------
dxX = xQ - xAA;
dyY = yQ - yAA;
dD = hypot(dxX, dyY);  % ??? sqrt(dx^2 + dy^2)
theta_dd = atan2(-0.42,-0.91)+deg2rad(360);

% ------- 3. ???? ??????? ? -------
cos_betaa = (r3^2 + dD^2 - r4^2) / (2 * r3 * dD);
cos_betaa = min(max(cos_betaa, -1), 1);  % ???? ??? ????
betaA = acos(cos_betaa);  % ?????????
betaa_deg = rad2deg(betaA);

% ------- 4. ???? ?3 -------
theta33 = theta_dd - betaA;
theta33_deg = mod(rad2deg(theta33), 360);  % ????? ???????

% ------- 5. ???? ?4 -------
% ???????? ????? ?????? ??????
sin_theta44 = (r3 * sin(theta33) - dD * sin(theta_dd)) / r4;
cos_theta44 = (r3 * cos(theta33) - dD * cos(theta_dd)) / r4;
theta44 = atan2(sin_theta44, cos_theta44);
theta44_deg = mod(rad2deg(theta44), 360);

%% V&A lEP %%
% V %
% ?????? 1: ???? ?????? ??? ?????? A
VAxx = -omega2 * r2 * sin(theta22);
VAyy =  omega2 * r2 * cos(theta22);

% ?????? 2: ???? omega3
omega33 = (VAxx * cos(theta44) + VAyy * sin(theta44)) / (r3 * sin(theta33 - theta44));

% ?????? 3: ???? omega4
omega44 = (-VAxx * sin(theta44) + VAyy * cos(theta44) + r3 * omega33 * cos(theta33 - theta44)) / r4;

% A %
aa_AX=-omega2^2*r2*cos(theta22)-alpha2*r2*sin(theta22);
aa_AY=-omega2^2*r2*sin(theta22)+alpha2*r2*cos(theta22);
alpha33=((omega44^2*r4)+(aa_AX*cos(theta44))+(aa_AY*sin_theta44)-(omega33^2*r3*cos(theta33-theta44)))/(r3*sin(theta33-theta44));
alpha44=(-(aa_AX*sin(theta44))+(aa_AY*cos(theta44))-(omega33^2*r3*sin(theta33-theta44))+(alpha33*r3*cos(theta33-theta44)))/(r4);

%% === Force & Torque Analysis lep ===
% ????? (????)
m2 = 4.13; m3 = 6.2; m4 = 3.42;

% ?????? (??*??^2)
I2 = 8496.82; I3 = 18438.11; I4 = 5089.54;

% ??????
Tt_resist = 3e-5; % N∑m

% ??????? ??? SI
m2 = m2 * 1e-3; m3 = m3 * 1e-3; m4 = m4 * 1e-3;
I2 = I2 * 1e-9; I3 = I3 * 1e-9; I4 = I4 * 1e-9;
r2 = r2 * 1e-3; r3 = r3 * 1e-3; r4 = r4 * 1e-3;

% ??????? ??????
aag2x=-omega2^2*(r2/2)*cos(theta22)-alpha2*(r2/2)*sin(theta22);
aag2y=-omega2^2*(r2/2)*sin(theta22)+alpha2*(r2/2)*cos(theta22);

aag3x=aa_AX/1000+(r3/2)*((-omega33^2)*cos(theta33)-alpha33*sin(theta33));
aag3y=aa_AY/1000+(r3/2)*((-omega33^2)*sin(theta33)-alpha33*cos(theta33));

aag4x=(r4/2)*(-omega44^2*cos(theta44)-alpha44*sin(theta44));
aag4y=(r4/2)*(-omega44^2*sin(theta44)+alpha44*cos(theta44));

%Inertia Forces
xx2=-m2*aag2x;   yy2=-m2*aag2y;
xx3=-m3*aag3x;   yy3=-m3*aag3y;
xx4=-m4*aag4x;   yy4=-m4*aag4y;
%T
syms XX43 YY43;  
eq1 =-yy3*(r3/2)*cos(theta33)-YY43*r3*cos(theta33)+xx3*(r3/2)*sin(theta33)+XX43*r3*sin(theta33);  % ???????? ??????
eq2 =Tt_resist-xx4*(r4/2)*sin(3.14159265-theta44)-yy4*(r4/2)*cos(3.14159265-theta44)+XX43*r4*sin(3.14159265-theta44)+YY43*r4*cos(3.14159265-theta44);  % ???????? ???????
sol = solve([eq1, eq2], [XX43, YY43]); 
xx43_val = double(sol.XX43);
yy43_val = double(sol.YY43);
xx23=-xx43_val-xx3;
yy23=-yy43_val-yy3;
MM=xx2*(r2/2)*sin(theta22)-yy2*(r2/2)*cos(theta22)-xx23*r2*sin(theta22)+yy23*r2*cos(theta22);

%% «› —÷ «·„ €Ì—«  „ÊÃÊœ… „”»ﬁ« „À·:
% theta2_deg, theta3_deg, theta4_deg, omega2, omega3, omega4, alpha2, alpha3, alpha4, M
% theta22_deg, theta33_deg, theta44_deg, omega33, omega44, alpha33, alpha44, MM

% ’› 1: REP
row = 1;
results.Condition{row} = 'REP';
results.Theta2(row)     = theta2_deg;
results.Theta3(row)     = theta3_deg;
results.Theta4(row)     = theta4_deg;
results.Omega2(row)     = omega2;
results.Omega3(row)     = omega3;
results.Omega4(row)     = omega4;
results.Alpha2(row)     = alpha2;
results.Alpha3(row)     = alpha3;
results.Alpha4(row)     = alpha4;
results.M12(row)        = M;

% ’› 2: LEP
row = row + 1;
results.Condition{row} = 'LEP';
results.Theta2(row)     = theta22_deg;
results.Theta3(row)     = theta33_deg;
results.Theta4(row)     = theta44_deg;
results.Omega2(row)     = omega2;
results.Omega3(row)     = omega33;
results.Omega4(row)     = omega44;
results.Alpha2(row)     = alpha2;
results.Alpha3(row)     = alpha33;
results.Alpha4(row)     = alpha44;
results.M12(row)        = MM;

% «·ÊÕœ« 
units.Condition = 'none';
units.Theta2    = 'degrees';
units.Theta3    = 'degrees';
units.Theta4    = 'degrees';
units.Omega2    = 'rad/s';
units.Omega3    = 'rad/s';
units.Omega4    = 'rad/s';
units.Alpha2    = 'rad/s^2';
units.Alpha3    = 'rad/s^2';
units.Alpha4    = 'rad/s^2';
units.M12       = 'Nm';

% »‰«¡ struct array ··ÃœÊ·
nRows = length(results.Condition);
resultsStructArray(nRows) = struct();

for i = 1:nRows
    resultsStructArray(i).Condition = results.Condition{i};
    resultsStructArray(i).Theta2    = results.Theta2(i);
    resultsStructArray(i).Theta3    = results.Theta3(i);
    resultsStructArray(i).Theta4    = results.Theta4(i);
    resultsStructArray(i).Omega2    = results.Omega2(i);
    resultsStructArray(i).Omega3    = results.Omega3(i);
    resultsStructArray(i).Omega4    = results.Omega4(i);
    resultsStructArray(i).Alpha2    = results.Alpha2(i);
    resultsStructArray(i).Alpha3    = results.Alpha3(i);
    resultsStructArray(i).Alpha4    = results.Alpha4(i);
    resultsStructArray(i).M12       = results.M12(i);
end

%  ÕÊÌ· ·ÃœÊ·
resultsTable = struct2table(resultsStructArray);

%  ÃÂÌ“ √”„«¡ «·√⁄„œ… „⁄ «·ÊÕœ« 
varNames = resultsTable.Properties.VariableNames;
unitNames = cell(size(varNames));
for k = 1:length(varNames)
    if isfield(units, varNames{k})
        unitNames{k} = units.(varNames{k});
    else
        unitNames{k} = '';
    end
end

colNamesWithUnits = strcat(varNames, " [", unitNames, "]");
resultsTable.Properties.VariableNames = matlab.lang.makeValidName(colNamesWithUnits);

% ⁄—÷ «·ÃœÊ· ›Ì figure Ê«Õœ
f = figure('Name','Results Table','NumberTitle','off', 'Position',[100 100 900 300]);

t = uitable('Parent', f, ...
            'Data', table2cell(resultsTable), ...
            'ColumnName', resultsTable.Properties.VariableNames, ...
            'Units', 'normalized', ...
            'Position', [0 0 1 1], ...
            'FontSize', 12);

%% plot %% 
function data = read_csv_matlab(filename)
    % «· Õﬁﬁ „‰ ÊÃÊœ «·„·›
    if ~isfile(filename)
        error('File not found: %s', filename);
    end

    % ﬁ—«¡… «·»Ì«‰«  „‰ «·„·›
    data = readtable(filename);

    % ⁄—÷ »⁄÷ «·„⁄·Ê„«  «·√”«”Ì…
    disp('Data preview:');
    disp(head(data));

    % —”„ «·»Ì«‰«  (√Ê· ⁄„Êœ ÂÊ Time° Ê»«ﬁÌ «·√⁄„œ… »Ì«‰« )
    time = data{:, 1};
    numCols = size(data, 2);

    % ≈⁄œ«œ «·‘ﬂ·
    figure('Color', [0.9, 0.9, 0.9]); % Œ·›Ì… —„«œÌ… ›« Õ…
    hold on;
    colors = lines(numCols - 1); % √·Ê«‰ „ ‰«”ﬁ… ·ﬂ· Œÿ

    for i = 2:numCols
        plot(time, data{:, i}, 'LineWidth', 1.5, 'Color', colors(i-1, :), 'DisplayName', data.Properties.VariableNames{i});
    end

    xlabel('Time', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('DEG', 'FontSize', 12, 'FontWeight', 'bold');
    title(' theta Plot', 'FontSize', 14, 'FontWeight', 'bold');
    legend('show', 'Location', 'bestoutside');
    grid on;
    grid minor;
    box on;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.2);
    hold off;
end



function data = read_csvS_matlab(filename)
    if ~isfile(filename)
        error('File not found: %s', filename);
    end

    data = readtable(filename);

    
    disp('Data preview:');
    disp(head(data));

   
    time = data{:, 1};
    numCols = size(data, 2);

    % ≈⁄œ«œ «·‘ﬂ·
    figure('Color', [0.9, 0.9, 0.9]); 
    hold on;
    colors = lines(numCols - 1);

    for i = 2:numCols
        plot(time, data{:, i}, 'LineWidth', 1.5, 'Color', colors(i-1, :), 'DisplayName', data.Properties.VariableNames{i});
    end

    xlabel('Time', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('DEG/S', 'FontSize', 12, 'FontWeight', 'bold');
    title(' omega Plot', 'FontSize', 14, 'FontWeight', 'bold');
    legend('show', 'Location', 'bestoutside');
    grid on;
    grid minor;
    box on;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.2);
    hold off;
end


function data = read_csvSS_matlab(filename)
    if ~isfile(filename)
        error('File not found: %s', filename);
    end

    data = readtable(filename);

    % ⁄—÷ »⁄÷ «·„⁄·Ê„«  «·√”«”Ì…
    disp('Data preview:');
    disp(head(data));

    % —”„ «·»Ì«‰«  (√Ê· ⁄„Êœ ÂÊ Time° Ê»«ﬁÌ «·√⁄„œ… »Ì«‰« )
    time = data{:, 1};
    numCols = size(data, 2);

    % ≈⁄œ«œ «·‘ﬂ·
    figure('Color', [0.9, 0.9, 0.9]); % Œ·›Ì… —„«œÌ… ›« Õ…
    hold on;
    colors = lines(numCols - 1); % √·Ê«‰ „ ‰«”ﬁ… ·ﬂ· Œÿ

    for i = 2:numCols
        plot(time, data{:, i}, 'LineWidth', 1.5, 'Color', colors(i-1, :), 'DisplayName', data.Properties.VariableNames{i});
    end

    xlabel('Time', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('DEG/S^2', 'FontSize', 12, 'FontWeight', 'bold');
    title(' alpha Plot', 'FontSize', 14, 'FontWeight', 'bold');
    legend('show', 'Location', 'bestoutside');
    grid on;
    grid minor;
    box on;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.2);
    hold off;
end
