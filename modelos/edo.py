import numpy as np
from scipy.integrate import solve_ivp

def buck_PV_linear_modelo_medio(t, y, Vbat, Iph, L, Cin, Rp):
  iL = y[0]
  vpv = y[1]

  if t < 0.06:
    duty = 0.3
  elif t > 0.06 and t <0.12:
    duty = 0.5
  elif t > 0.12 and t <0.18:
    duty = 0.4
  elif t > 0.18 and t <0.24:
    duty = 0.6
  else:
    duty = 0.4

  d_dt_iL = duty/L * vpv - Vbat/L
  d_dt_vpv = Iph/Cin - 1/(Rp*Cin) * vpv - duty/Cin * iL

  return np.array([d_dt_iL, d_dt_vpv])