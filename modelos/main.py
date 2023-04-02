"""
Arquivo main para testar códigos dos modelos dos conversores
Trabalho Futuro - Refatorar implementando TDD
"""
import matplotlib.pyplot as plt
import conversores_cc_basicos
import numpy as np

conversor = conversores_cc_basicos.Buck_Boost_ideal(1.96, 375e-6, 0.146e-3, 24)
duty = list(np.arange(0,1,0.1))
T = np.arange(0,0.1,0.0001)
U = np.ones_like(T)
ys_TS = conversor.forced_response_TS(duty, T, U, 0., False, False, None, True)

print(ys_TS.shape)
#---------------------------------------------------------------------------------
# Plotando o resultado
fig1, f1_axes = plt.subplots(ncols=1, nrows=2, figsize=(15,10))
f1_axes[0].plot(T, ys_TS[0], linewidth=1.5, label="Modelo fuzzy TS")
f1_axes[0].set_title("Tensã de Saída (V)")
f1_axes[0].set_xlabel("Tempo (s)")
f1_axes[0].set_ylabel("Tensão (V)")
f1_axes[0].legend()
f1_axes[1].plot(T, ys_TS[1], linewidth=1.5, label="Modelo fuzzy TS")
f1_axes[1].set_title("Corrente de Saída (A)")
f1_axes[1].set_xlabel("Tempo (s)")
f1_axes[1].set_ylabel("Corrente (A)")
f1_axes[1].legend()
plt.show()