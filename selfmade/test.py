from scipy.integrate import odeint
import numpy as np

def f(t, y):
  return t**2 + y

t_span = (0, 5)
y0 = 1
t_eval = np.linspace(0, 5, 101)

y = odeint(f, y0, t_span)

# Tracer la solution
import matplotlib.pyplot as plt

plt.plot(t_eval, y[:, 0])  # On extrait la première colonne de y (correspond à y)
plt.xlabel('t')
plt.ylabel('y')
plt.show()