import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np
import pandas as pd
import edo

<<<<<<< HEAD
# # Simulação do Modelo Fuzzy T-S com passo 1E-5

# conversor = conversores_cc_basicos.Buck_ideal_entrada(Vo=12, L=700e-6, C=500e-6, Iin=10, Rp=10)
=======
conversor = conversores_cc_basicos.Buck_ideal_entrada(Vo=12, L=700e-6, C=500e-6, Iin=10, Rp=10)
>>>>>>> c250c0896e94b9e3199abbe0f2b475bee425a474

T = np.arange(0,0.3,1e-5)
U = np.ones_like(T)

duty = []
for i in range(int(len(T)/5)):
    duty.append(0.3)
for i in range(int(len(T)/5)):
     duty.append(0.5)
for i in range(int(len(T)/5)):
     duty.append(0.4)
for i in range(int(len(T)/5)):
     duty.append(0.6)
for i in range(int(len(T)/5)):
     duty.append(0.4)

# ys_TS = conversor.forced_response_TS(duty, 0.24, 0.6, T, U, 0., False, False, None, True)

# Simulação do PSIM com passo 1E-6 ---------------------------------------------------------------

df = pd.read_fwf('buck.txt')
#print(df.head())

tempo = df[['Time']].to_numpy()
corrente_indutor = df[['I(RL1)']].to_numpy()
tensao_entrada = df[['vin']].to_numpy()


tempo = tempo.transpose()
corrente_indutor = corrente_indutor.transpose()
tensao_entrada = tensao_entrada.transpose()

tempo = tempo[0][0:-1:10]
corrente_indutor = corrente_indutor[0][0:-1:10]
tensao_entrada = tensao_entrada[0][0:-1:10]

## Simulação das EDOs do sistema
C0 = np.array([0, 0])

t_limites = np.array([0, 0.3])
t_passo = 1e-7
times = np.arange(t_limites[0], t_limites[1]+t_passo, t_passo)

rtol = 1e-6

solution = edo.solve_ivp(edo.buck_PV_linear_modelo_medio, t_limites, C0, t_eval = times, rtol=rtol, args=(12, 10, 100e-6, 500e-6, 10))
t = solution.t
iL = solution.y[0]
vpv = solution.y[1]


## Valores estáticos de referência
Vin_t = []
il_t = []
for D in duty:
    Vin_t.append(12/D)
    il_t.append(10/D - 12/(10*D**2))
#---------------------------------------------------------------------------------
# Plotando o resultado
fig1, f1_axes = plt.subplots(ncols=1, nrows=3, figsize=(15,10))
f1_axes[0].grid(True); f1_axes[1].grid(True); f1_axes[2].grid(True)
f1_axes[0].plot(tempo, tensao_entrada, linewidth=1.5, label="$v_{pv}$ - Simulação PSIM")
# f1_axes[0].plot(T, ys_TS[0], linewidth=1.5, label="'$v_{pv}$ - Modelo fuzzy TS")
f1_axes[0].plot(t, vpv, linewidth=1.5, label='$v_{pv}$ - EDOs Não Lineares')
f1_axes[0].plot(T, Vin_t, linewidth=1.5, label="Referência de tensão estacionária", linestyle='dashed')
f1_axes[0].set_xlabel("Tempo (s)")
f1_axes[0].set_ylabel("Tensão de Entrada do Conversor - $v_{pv}$ (V)")
f1_axes[0].legend()
f1_axes[1].grid(True)
f1_axes[1].plot(tempo, corrente_indutor, linewidth=1.5, label="$i_L$ - Simulação PSIM")
# f1_axes[1].plot(T, ys_TS[1], linewidth=1.5, label="'$i_L$ - Modelo fuzzy TS")
f1_axes[1].plot(t, iL, linewidth=1.5, label="$i_L$ - EDOs Não Lineares")
f1_axes[1].plot(T, il_t, linewidth=1.5, label="Referência de corrente estacionária", linestyle='dashed')
f1_axes[1].set_xlabel("Tempo (s)")
f1_axes[1].set_ylabel("Corrente no Indutor - $i_L$ (A)")
f1_axes[1].legend()
f1_axes[2].plot(T, duty, linewidth=1.5, label="$d(t)$")
f1_axes[2].set_xlabel("Tempo (s)")
f1_axes[2].set_ylabel("Duty-Cycle - $d(t)$")
f1_axes[2].legend()
plt.show()