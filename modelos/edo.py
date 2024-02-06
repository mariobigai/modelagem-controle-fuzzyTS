import numpy as np
from scipy.integrate import solve_ivp

def buck_PV_linear(t, y, Vbat, Iph, L, Cin, Rp):
  iL = y[0]
  vpv = y[1]

  if t < 0.06:
    duty = 0.6
  elif t > 0.06 and t <0.12:
    duty = 0.51
  elif t > 0.12 and t <0.18:
    duty = 0.42
  elif t > 0.18 and t <0.24:
    duty = 0.33
  else:
    duty = 0.24

  d_dt_iL = duty/L * vpv - Vbat/L
  d_dt_vpv = Iph/Cin - 1/(Rp*Cin) * vpv - duty/Cin * iL

  return np.array([d_dt_iL, d_dt_vpv])

def buckboost_R(t, y, L, C, R, Vin):
  iL = y[0]
  vo = y[1]

  if t < 0.02:
    duty = 0.3
  elif t > 0.02 and t < 0.04:
    duty = 0.5
  elif t > 0.04 and t < 0.06:
    duty = 0.4
  elif t > 0.06 and t <0.08:
    duty = 0.6
  else:
    duty = 0.4

  d_dt_iL = -(1-duty)/L * vo + Vin/L*duty
  d_dt_vo = (1-duty)/C * iL - 1/(R*C)*vo

  return np.array([d_dt_iL, d_dt_vo])