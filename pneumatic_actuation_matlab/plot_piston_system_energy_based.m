%% Configuration
% Assumptions: planar segment

l_p = 1; % length of piston [m]
R_p = 0.1; % radius of segment [m]
mu_p0 = 0.5 * l_p; % initial position of piston [m]

p_atm = 1*10^5; % atmospheric pressure [Pa]

% plotting 
figureWidth = 720; % 
figureHeight = 480; %

%% Symbolic derivation
% symbolic volume function
syms V_p(mu_p);

V_p(mu_p) = mu_p * pi * R_p^2;

% gradient of volume with respect to piston position
dV_p_dmu_p = pi * R_p^2;

% symbolic conservative torque function
syms G_P_mu_p(mu_p, p);
G_P_mu_p(mu_p, p) = dV_p_dmu_p*p_atm ...
                    + dV_p_dmu_p*p*(log(V_p(mu_p0)/V_p(mu_p))-1);

%% Plotting of conservative force
f = figure('Name', 'Piston system: Energy-based approach');
grid on
box on
set(gcf,'color','w');
f.Position(3:4) = [figureWidth figureHeight];
hold on
syms G_P_mu_p_const_p(mu_p)
for p=0:0.5*10^5:4*10^5
    G_P_mu_p_const_p(mu_p) = G_P_mu_p(mu_p, p);

    plot_name = 'p='+string(p*10^(-5))+' [bar]';
    fplot(G_P_mu_p_const_p(mu_p), [0 l_p], DisplayName=plot_name, LineWidth=1.5)
    xlabel("$\mu_\mathrm{p}$ [m]", Interpreter="latex", FontSize=13)
    ylabel("$G_{\mathrm{P}}^{\mu_\mathrm{p}}$ [N]", Interpreter="latex", FontSize=13)
    
    hold on;
end
legend(FontSize=11)
hold off;
