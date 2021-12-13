%% Configuration
% Assumptions: planar segment

L_0 = 110*10^(-3); % inextended length of segment [m]
d = 21*10^(-3); % radius of segment used for parametrization q [m]
R_C_in = 7.14*10^(-3); % inner chamber wall radius [m]
R_C_out = 20.19*10^(-3); % outer chamber wall radius [m]
b_C = 8.7*10^(-3); % modelled planar depth [m]
eta_rib = 0.6;  % chamber volume efficiency (reduced by rib-like structure) [-]

p_atm = 1*10^5; % atmospheric pressure [Pa]
Delta_max = 45/180*pi*d; % maximum bending angle we plot [m]

% plotting 
figureWidth = 720; % 
figureHeight = 480; %

%% Symbolic derivation
% symbolic volume function
syms V_C_L(Delta, delta_L) V_C_R(Delta, delta_L);

V_C_L(Delta, delta_L) = eta_rib*b_C*(R_C_out-R_C_in)*(L_0 + delta_L) ...
                      - eta_rib*b_C*(R_C_out^2 - R_C_in^2)/2*Delta/d;
V_C_R(Delta, delta_L) = eta_rib*b_C*(R_C_out-R_C_in)*(L_0 + delta_L) ...
                      + eta_rib*b_C*(R_C_out^2 - R_C_in^2)/2*Delta/d;

% gradient of volume with respect to configuration
dV_dDelta_L = - eta_rib * b_C * (R_C_out^2 - R_C_in^2) / (2 * d);
dV_dDelta_R = + eta_rib * b_C * (R_C_out^2 - R_C_in^2) / (2 * d);
dV_ddeltaL = eta_rib * b_C * (R_C_out - R_C_in);

% symbolic conservative torque function
syms G_P_q_L(Delta, delta_L, p) G_P_q_L(Delta, delta_L, p);
G_P_q_L(Delta, delta_L, p) = dV_dDelta_L*p_atm ...
                           + dV_dDelta_L*p*(log(V_C_L(0,0)/V_C_L(Delta, delta_L))-1);
G_P_q_R(Delta, delta_L, p) = dV_dDelta_R*p_atm ...
                           + dV_dDelta_R*p*(log(V_C_R(0,0)/V_C_R(Delta, delta_L))-1);

%% Plotting of chamber volume
f = figure('Name', 'Chamber volume');
grid on
box on
set(gcf,'color','w');
f.Position(3:4) = [figureWidth figureHeight];
syms V_C_L_const_delta_L(Delta) V_C_R_const_delta_L(Delta)
V_C_L_const_delta_L(Delta) = V_C_L(Delta, 0);
V_C_R_const_delta_L(Delta) = V_C_R(Delta, 0);
fplot(V_C_L_const_delta_L(Delta), [-Delta_max Delta_max], DisplayName="V_{C,L}", LineWidth=1.5)
hold on;
fplot(V_C_R_const_delta_L(Delta), [-Delta_max Delta_max], DisplayName="V_{C,R}", LineWidth=1.5)
hold on;
xlabel("$\Delta_i$ [m]", Interpreter="latex", FontSize=13)
ylabel("$V_{\mathrm{C}} [m^3]$", Interpreter="latex", FontSize=13)
legend(FontSize=11)
hold off;

%% Plotting of conservative force
% Assumption: inextensible

f = figure('Name', 'Energy-based approach left chamber');
grid on
box on
set(gcf,'color','w');
f.Position(3:4) = [figureWidth figureHeight];
hold on
syms G_P_q_L_const_p(Delta)
for p=0:0.5*10^5:4*10^5
    G_P_q_L_const_p(Delta) = G_P_q_L(Delta, 0, p);

    plot_name = 'p='+string(p*10^(-5))+' [bar]';
    fplot(G_P_q_L_const_p(Delta), [-Delta_max Delta_max], DisplayName=plot_name, LineWidth=1.5)
    xlabel("$\Delta_i$ [m]", Interpreter="latex", FontSize=13)
    ylabel("$G_{\mathrm{P},\mathrm{L}}^q$ [N]", Interpreter="latex", FontSize=13)
    
    hold on;
end
legend(FontSize=11)
hold off;

f = figure('Name', 'Energy-based approach right chamber');
grid on
box on
set(gcf,'color','w');
f.Position(3:4) = [figureWidth figureHeight];
hold on
syms G_P_q_R_const_p(Delta)
for p=0:0.5*10^5:4*10^5
    G_P_q_R_const_p(Delta) = G_P_q_R(Delta, 0, p);

    plot_name = 'p='+string(p*10^(-5))+' [bar]';
    fplot(G_P_q_R_const_p(Delta), [-Delta_max Delta_max], DisplayName=plot_name, LineWidth=1.5)
    xlabel("$\Delta_i$ [m]", Interpreter="latex", FontSize=13)
    ylabel("$G_{\mathrm{P},\mathrm{R}}^q$ [N]", Interpreter="latex", FontSize=13)
    
    hold on;
end
legend(FontSize=11)
hold off;

%% Singularity investigation
Delta_star_L = 2*(1-exp(-1))*(R_C_out-R_C_in)/(R_C_out^2-R_C_in^2)*L_0*d
theta_star_L = Delta_star_L / d / pi * 180

