import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np
import pandas as pd
import edo

# Simulação do PSIM com passo 1E-6 ---------------------------------------------------------------

df = pd.read_fwf('simu/buck_PV_ajustado.txt')
#print(df.head())

tempo = df[['Time']].to_numpy()
corrente_indutor = df[['I(RL1)']].to_numpy()
tensao_entrada = df[['vin']].to_numpy()
duty = df[['D']].to_numpy()


tempo = tempo.transpose()
corrente_indutor = corrente_indutor.transpose()
tensao_entrada = tensao_entrada.transpose()
duty = duty.transpose()

tempo = tempo[0][0:-1:10]
corrente_indutor = corrente_indutor[0][0:-1:10]
tensao_entrada = tensao_entrada[0][0:-1:10]
duty = duty[0][0:-1:10]

# # Simulação do Modelo Fuzzy T-S com passo 1E-5
# conversor = conversores_cc_basicos.Buck_PV(Vo=12, L=22.109e-6, C=2.7e-3, Iin=7.5, Rp=8)

# T = np.arange(0,0.3,1e-5)
# U = np.ones_like(T)
#
# duty = []
# for i in range(int(len(T)/5)):
#     duty.append(0.6)
# for i in range(int(len(T)/5)):
#      duty.append(0.51)
# for i in range(int(len(T)/5)):
#      duty.append(0.42)
# for i in range(int(len(T)/5)):
#      duty.append(0.33)
# for i in range(int(len(T)/5)):
#      duty.append(0.24)

# ys_TS = conversor.forced_response_TS(duty, 0.24, 0.6, il_simu=corrente_indutor, vcin_simu=tensao_entrada,
#                                      T=T, U=U)

# ## Simulação das EDOs do sistema
# C0 = np.array([0, 0])
#
# t_limites = np.array([0, 0.3])
# t_passo = 1e-7
# times = np.arange(t_limites[0], t_limites[1]+t_passo, t_passo)
#
# rtol = 1e-6
#
# solution = edo.solve_ivp(edo.buck_PV_linear, t_limites, C0, t_eval = times, rtol=rtol, args=(12, 19, 22.109e-6, 2.7e-3, 8))
# t = solution.t
# iL = solution.y[0]
# vpv = solution.y[1]
#
#
# ## Valores estáticos de referência
# Vin_t = []
# il_t = []
# for D in duty:
#     Vin_t.append(12/D)
#     il_t.append(10/D - 12/(10*D**2))
#---------------------------------------------------------------------------------
# Plotando o resultado
fig1, f1_axes = plt.subplots(ncols=1, nrows=3, figsize=(15,10))
f1_axes[0].grid(True); f1_axes[1].grid(True); f1_axes[2].grid(True)
f1_axes[0].plot(tempo, tensao_entrada, linewidth=1.5, label="$v_{pv}$ - Simulação PSIM")
# f1_axes[0].plot(T, ys_TS[0], linewidth=1.5, label="'$v_{pv}$ - Modelo fuzzy TS")
# f1_axes[0].plot(t, vpv, linewidth=1.5, label='$v_{pv}$ - EDOs Não Lineares')
# f1_axes[0].plot(T, Vin_t, linewidth=1.5, label="Referência de tensão estacionária", linestyle='dashed')
f1_axes[0].set_xlabel("Tempo (s)")
f1_axes[0].set_ylabel("Tensão de Entrada do Conversor - $v_{pv}$ (V)")
f1_axes[0].legend()
f1_axes[1].grid(True)
f1_axes[1].plot(tempo, corrente_indutor, linewidth=1.5, label="$i_L$ - Simulação PSIM")
# f1_axes[1].plot(T, ys_TS[1], linewidth=1.5, label="'$i_L$ - Modelo fuzzy TS")
# f1_axes[1].plot(t, iL, linewidth=1.5, label="$i_L$ - EDOs Não Lineares")
# f1_axes[1].plot(T, il_t, linewidth=1.5, label="Referência de corrente estacionária", linestyle='dashed')
f1_axes[1].set_xlabel("Tempo (s)")
f1_axes[1].set_ylabel("Corrente no Indutor - $i_L$ (A)")
f1_axes[1].legend()
f1_axes[2].plot(tempo, duty, linewidth=1.5, label="$d(t)$")
f1_axes[2].set_xlabel("Tempo (s)")
f1_axes[2].set_ylabel("Duty-Cycle - $d(t)$")
f1_axes[2].legend()
plt.show()