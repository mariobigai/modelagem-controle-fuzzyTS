import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np
import pandas as pd
import edo

# Simulação do PSIM com passo 1E-7 ---------------------------------------------------------------

df = pd.read_fwf('simu/buck_PV_projeto.txt')
#print(df.head())

t_PSIM = df[['Time']].to_numpy()
iL_PSIM = df[['I(RL1)']].to_numpy()
vpv_PSIM = df[['vin']].to_numpy()


t_PSIM = t_PSIM.transpose()
iL_PSIM = iL_PSIM.transpose()
vpv_PSIM = vpv_PSIM.transpose()

# Reduzindo o passo para 1E-5
t_PSIM = t_PSIM[0][0:-1:100]
iL_PSIM = iL_PSIM[0][0:-1:100]
vpv_PSIM = vpv_PSIM[0][0:-1:100]

## Simulação das EDOs do sistema ---------------------------------------------------------------
C0 = np.array([0, 0])
t_limites = np.array([0, 0.3])
t_passo = 1e-7
times = np.arange(t_limites[0], t_limites[1]+t_passo, t_passo)
rtol = 1e-6
                                                                                        #args=(Vbat, Iph, L, Cin, Rp)
solution = edo.solve_ivp(edo.buck_PV_linear, t_limites, C0, t_eval = times, rtol=rtol, args=(12, 19, 22.109e-6, 2.7e-3, 8))
t_EDO = solution.t
iL_EDO = solution.y[0]
vpv_EDO = solution.y[1]

# Simulação do Modelo Fuzzy T-S com passo 1E-5 -----------------------------------------
conversor = conversores_cc_basicos.Buck_PV(Vo=12, L=22.109e-6, C=2.7e-3, Iin=19, Rp=8)

t_TS = t_PSIM
duty = []
for i in range(int(len(t_TS)/5)):
    duty.append(0.6)
for i in range(int(len(t_TS)/5)):
     duty.append(0.51)
for i in range(int(len(t_TS)/5)):
     duty.append(0.42)
for i in range(int(len(t_TS)/5)):
     duty.append(0.33)
for i in range(int(len(t_TS)/5)):
     duty.append(0.24)

ys_TS = conversor.forced_response_TS(duty, 0.24, 0.6, il_simu=iL_PSIM, vcin_simu=vpv_PSIM, T=t_TS)
vpv_TS = ys_TS[0]
iL_TS = ys_TS[1]

#---------------------------------------------------------------------------------
# Plotando o resultado

# Tirando o transitório de partida - simulação começa em 0.05
t_PSIM = t_PSIM[5000:-1]; vpv_PSIM = vpv_PSIM[5000:-1]; iL_PSIM = iL_PSIM[5000:-1]
t_EDO = t_EDO[500000:-1]; vpv_EDO = vpv_EDO[500000:-1]; iL_EDO = iL_EDO[500000:-1]
t_TS = t_TS[5000:-1]; vpv_TS = vpv_TS[5000:-1]; iL_TS = iL_TS[5000:-1]
duty = duty[5000:-1]

fig1, f1_axes = plt.subplots(ncols=1, nrows=3, figsize=(15,10))
f1_axes[0].grid(True); f1_axes[1].grid(True); f1_axes[2].grid(True)
f1_axes[0].plot(t_PSIM, vpv_PSIM, linewidth=1.5, label="$v_{pv}$ - Simulação PSIM")
f1_axes[0].plot(t_TS, vpv_TS, linewidth=1.5, label="$v_{pv}$ - Modelo fuzzy TS")
f1_axes[0].plot(t_EDO, vpv_EDO, linewidth=1.5, label='$v_{pv}$ - EDOs Não Lineares')
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