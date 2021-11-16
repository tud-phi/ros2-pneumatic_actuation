% Assumptions: inextensible, planar segment

L_0 = 110*10^(-3);
d = 21*10^(-3);
R_C_in = 7.14*10^(-3);
R_C_out = 20.19*10^(-3);
b_C = 0.0087;
eta_rib = 0.6;

p_atm = 1*10^5;
Delta_max = 45/180*pi*d;

% symbolic volume function
syms V_C_L(Delta, delta_L) V_C_R(Delta, delta_L);

V_C_L(Delta, delta_L) = eta_rib*(R_C_out-R_C_in)*(L_0 + delta_L) ...
                      - eta_rib*(R_C_out^2 - R_C_in^2)/2*Delta/d;
V_C_R(Delta, delta_L) = eta_rib*(R_C_out-R_C_in)*(L_0 + delta_L) ...
                      + eta_rib*(R_C_out^2 - R_C_in^2)/2*Delta/d;

% gradient of volume with respect to configuration
dV_dDelta_L = - eta_rib * (R_C_out^2 - R_C_in^2) / (2 * d);
dV_dDelta_R = + eta_rib * (R_C_out^2 - R_C_in^2) / (2 * d);
dV_ddeltaL = eta_rib * (R_C_out - R_C_in);

% symbolic conservative torque function
syms G_P_q_L(Delta, delta_L, p) G_P_q_L(Delta, delta_L, p);
G_P_q_L(Delta, delta_L, p) = dV_dDelta_L*p_atm ...
                           + dV_dDelta_L*p*(log(V_C_L(0,0)/V_C_L(Delta, delta_L))-1);
G_P_q_R(Delta, delta_L, p) = dV_dDelta_R*p_atm ...
                           + dV_dDelta_R*p*(log(V_C_R(0,0)/V_C_R(Delta, delta_L))-1);

f = figure('Name', 'Energy-based approach');
grid on
box on
set(gcf,'color','w');
figureWidth = 720; % 
figureHeight = 480; %
f.Position(3:4) = [figureWidth figureHeight];
hold on

syms G_P_q_L_const_p(Delta)
for p=0:0.5*10^5:4*10^5
    G_P_q_L_const_p(Delta) = G_P_q_L(Delta, 0, p);

    plot_name = 'p='+string(p*10^(-5))+' [bar]';
    fplot(G_P_q_L_const_p(Delta), [-Delta_max Delta_max], DisplayName=plot_name, LineWidth=1.5)
    xlabel("$\Delta_i$ [m]", Interpreter="latex")
    ylabel("$G_{\mathrm{P},\mathrm{L}}^q$ [N]", Interpreter="latex")
    
    hold on;
end
legend(FontSize=11)
hold off;

