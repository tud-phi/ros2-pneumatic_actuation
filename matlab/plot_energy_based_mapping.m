% Assumptions: inextensible, planar segment

L_0 = 1;
d = 1;
R_C_in = 0.1;
R_C_out = 1;
b_C = 1;
eta_rib = 0.6;

% symbolic volume function
syms V_C_L(Delta, delta_L);
syms V_C_R(Delta, delta_L);

V_C_L(Delta, delta_L) = eta_rib*(R_C_out-R_C_in)*(L_0 + delta_L) ...
                        - eta_rib*(R_C_out)
