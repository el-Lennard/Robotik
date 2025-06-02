import numpy as np
import matplotlib.pyplot as plt

P0 = [1.0, 1.0]
P1 = [2.0, 2.0]
P2 = [3.0, 2.5]
P3 = [4.0, 2.0]

plt.plot([P0[0],P1[0]], [P0[1],P1[1]])
plt.plot([P2[0],P3[0]], [P2[1],P3[1]])
plt.plot([P1[0],P2[0]], [P1[1],P2[1]]) #C0-stetig


plt.axis('equal')
plt.grid(True)
plt.show()
'''
'''
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

P0 = np.array([1.0, 1.0])
P1 = np.array([2.0, 2.0])
P2 = np.array([3.0, 2.5])
P3 = np.array([4.0, 2.0])
# Definierte Tangenten (müssen übereinstimmen für C1-Stetigkeit)
t1 = np.array([1, 1])  # Tangente an P1
t2 = np.array([1, -1])  # Gleiche Richtung für C1-Stetigkeit

# Hermite-Spline für C1-Stetigkeit
x_vals = np.array([P1[0], P2[0]])
y_vals = np.array([P1[1], P2[1]])
tangents = np.array([t1[1], t2[1]])

spline = CubicHermiteSpline(x_vals, y_vals, tangents)

# Berechnung des Splines
x_spline = np.linspace(P1[0], P2[0], 100)
y_spline = spline(x_spline)

plt.plot([P0[0], P1[0]], [P0[1], P1[1]], 'bo-')  # Erstes Segment
plt.plot(x_spline, y_spline, 'g-', label="C1-stetige Verbindung")  # Glatter Übergang
plt.plot([P2[0], P3[0]], [P2[1], P3[1]], 'ro-')  # Zweites Segment

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title("C1-Stetigkeit (Positions- und Tangentenstetigkeit)")
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
P0 = np.array([1.0, 1.0])
P1 = np.array([2.0, 2.0])
P2 = np.array([3.0, 2.5])
P3 = np.array([4.0, 2.0])

# Kubischer Spline für C2-Stetigkeit
x_vals = np.array([P1[0], P2[0]])
y_vals = np.array([P1[1], P2[1]])

spline = CubicSpline(x_vals, y_vals, bc_type=((1,1.0),(1,-0.5)))

# Berechnung des Splines
x_spline = np.linspace(P1[0], P2[0], 100)
y_spline = spline(x_spline)

plt.plot([P0[0], P1[0]], [P0[1], P1[1]], 'bo-')  # Erstes Segment
plt.plot(x_spline, y_spline, 'g-', label="C2-stetige Verbindung")  # Glatte Verbindung
plt.plot([P2[0], P3[0]], [P2[1], P3[1]], 'ro-')  # Zweites Segment

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title("C2-Stetigkeit (Zusätzlich Krümmungsstetigkeit)")
plt.show()