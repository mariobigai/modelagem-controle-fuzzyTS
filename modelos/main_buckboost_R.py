import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np
import pandas as pd
import edo

# Simulação do PSIM com passo 1E-6 ---------------------------------------------------------------

df = pd.read_fwf('simu/buck_boost.txt')
#print(df.head())

t_PSIM = df[['Time']].to_numpy()
iL_PSIM = df[['I(RL1)']].to_numpy()
vo_PSIM = df[['Vo']].to_numpy()

t_PSIM = t_PSIM.transpose()
iL_PSIM = iL_PSIM.transpose()
vo_PSIM = vo_PSIM.transpose()

# Reduzindo o passo para 1E-5
t_PSIM = t_PSIM[0][0:-1:10]
iL_PSIM = iL_PSIM[0][0:-1:10]
vo_PSIM = vo_PSIM[0][0:-1:10]

## Simulação das EDOs do sistema com passo 1E-5 ----------------------------------------------------
C0 = np.array([0, 0]) # Condições iniciais
t_limites = np.array([0, .1])
t_passo = 1e-5 #passo de simulação
times = np.arange(t_limites[0], t_limites[1]+ t_passo/10, t_passo)
rtol = 1e-6 #tolerância (solver)

solution = edo.solve_ivp(edo.buckboost_R, t_limites, C0, t_eval = times, rtol=rtol, args=(.148e-3, 375e-6, 1.96, 24)) #args = (L, C, R, Vin)
t_EDO = solution.t
iL_EDO = solution.y[0]
vo_EDO = solution.y[1]

# Simulação do Modelo Fuzzy T-S com passo 1E-5 ---------------------------------------------------
conversor = conversores_cc_basicos.BuckBoost_R(Vin=24, L=.148e-3, C=375e-6, R=1.96)
t_TS = t_PSIM
duty = []
for i in range(int(len(t_TS)/5)):
    duty.append(0.3)
for i in range(int(len(t_TS)/5)):
     duty.append(0.5)
for i in range(int(len(t_TS)/5)):
     duty.append(0.4)
for i in range(int(len(t_TS)/5)):
     duty.append(0.6)
for i in range(int(len(t_TS)/5)):
     duty.append(0.4)

ys_TS = conversor.forced_response_TS(duty, 0, 0.9, il_simu=iL_PSIM, vo_simu=vo_PSIM, T=t_TS)
vo_TS = ys_TS[0]
iL_TS = ys_TS[1]

#---------------------------------------------------------------------------------
# Plotando o resultado
fig1, f1_axes = plt.subplots(ncols=1, nrows=3, figsize=(15,10))
f1_axes[0].grid(True); f1_axes[1].grid(True); f1_axes[2].grid(True)
f1_axes[0].plot(t_PSIM, vo_PSIM, linewidth=1.5, label="$v_{o}$ - Simulação PSIM")
f1_axes[0].plot(t_TS, vo_TS, linewidth=1.5, label="$v_{o}$ - Modelo fuzzy TS")
f1_axes[0].plot(t_EDO, vo_EDO, linewidth=1.5, label='$v_{o}$ - EDOs Não Lineares')
f1_axes[0].set_xlabel("Tempo (s)")
f1_axes[0].set_ylabel("Tensão de Entrada do Conversor - $v_{pv}$ (V)")
f1_axes[0].legend()
f1_axes[1].grid(True)
f1_axes[1].plot(t_PSIM, iL_PSIM, linewidth=1.5, label="$i_L$ - Simulação PSIM")
f1_axes[1].plot(t_TS, iL_TS, linewidth=1.5, label="$i_L$ - Modelo fuzzy TS")
f1_axes[1].plot(t_EDO, iL_EDO, linewidth=1.5, label="$i_L$ - EDOs Não Lineares")
f1_axes[1].set_xlabel("Tempo (s)")
f1_axes[1].set_ylabel("Corrente no Indutor - $i_L$ (A)")
f1_axes[1].legend()
f1_axes[2].plot(t_TS, duty, linewidth=1.5, label="$d(t)$")
f1_axes[2].set_xlabel("Tempo (s)")
f1_axes[2].set_ylabel("Duty-Cycle - $d(t)$")
f1_axes[2].legend()
plt.show()