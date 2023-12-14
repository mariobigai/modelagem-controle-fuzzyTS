import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np
import pandas as pd

conversor = conversores_cc_basicos.Buck_ideal_entrada(Vo=12, L=22.109e-6, C=2.7e-6, Iin=7.5, Rp=8)

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
ys_TS = conversor.forced_response_TS(duty, 0.24, 0.6, T, U, 0., False, False, None, True)

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

Vin_t = []
il_t = []
for D in duty:
    Vin_t.append(12/D)
    il_t.append(10/D - 12/(10*D**2 ))
#---------------------------------------------------------------------------------
# Plotando o resultado
fig1, f1_axes = plt.subplots(ncols=1, nrows=2, figsize=(15,10))
f1_axes[0].grid(True)
#f1_axes[0].plot(tempo, tensao_entrada, linewidth=1.5, label="Simulação PSIM")
f1_axes[0].plot(T, ys_TS[0], linewidth=1.5, label="Modelo fuzzy TS")
#f1_axes[0].plot(T, Vin_t, linewidth=1.5, label="Referência de tensão estacionária", linestyle='dashed')
f1_axes[0].set_title("Tensão de Entrada do Conversor(V)")
f1_axes[0].set_xlabel("Tempo (s)")
f1_axes[0].set_ylabel("Tensão (V)")
f1_axes[0].legend(loc = 'upper right')
f1_axes[1].grid(True)
#f1_axes[1].plot(tempo, corrente_indutor, linewidth=1.5, label="Simulação PSIM")
f1_axes[1].plot(T, ys_TS[1], linewidth=1.5, label="Modelo fuzzy TS")
#f1_axes[1].plot(T, il_t, linewidth=1.5, label="Referência de corrente estacionária", linestyle='dashed')
f1_axes[1].set_title("Corrente do Indutor (A)")
f1_axes[1].set_xlabel("Tempo (s)")
f1_axes[1].set_ylabel("Corrente (A)")
f1_axes[1].legend(loc = 'upper right')
plt.show()