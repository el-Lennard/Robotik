import numpy as np
import matplotlib.pyplot as plt
from math import acos, degrees, sqrt, atan2, cos, sin

def calculate_angles(A, B, L1, L2, L3, tol=1e-6):
    """
    Berechnet die möglichen Winkelkonfigurationen für drei Linien mit festen Endpunkten A und B.
    
    Args:
        A (tuple): Startpunkt (x, y).
        B (tuple): Endpunkt (x, y).
        L1, L2, L3 (float): Längen der drei Linien.
        tol (float): Toleranz für numerische Genauigkeit.
    
    Returns:
        list: Liste von Lösungen, jede Lösung ist ein Tupel (theta1, theta2, theta3).
              Gibt eine leere Liste zurück, wenn keine Lösung existiert.
    """
    xA, yA = A
    xB, yB = B
    AB = sqrt((xB - xA)**2 + (yB - yA)**2)
    
    # Überprüfe, ob eine Lösung möglich ist
    if (L1 + L2 + L3 < AB - tol) or (abs(L1 - L2 - L3) > AB + tol) \
       or (abs(L2 - L1 - L3) > AB + tol) or (abs(L3 - L1 - L2) > AB + tol):
        return []  # Keine Lösung möglich
    
    # Fall: Alle Linien liegen auf einer Geraden (nur eine Lösung)
    if abs(L1 + L2 + L3 - AB) < tol:
        theta_AB = atan2(yB - yA, xB - xA)
        return [(theta_AB, theta_AB, theta_AB)]  # Alle Winkel gleich
    
    # Allgemeiner Fall: Zwei Lösungen (Spiegelungen)
    solutions = []
    
    # Berechne die Position des mittleren Gelenks (Punkt P)
    # Nutze den Cosinus-Satz im Dreieck L1-L2-AB3, wobei AB3 = AB - L3
    AB3 = AB
    L_total = L1 + L2 + L3
    if L_total > AB + tol:
        # Berechne mögliche Positionen für das erste Gelenk (P)
        # Wir lösen das 2D-Problem mit Kreis-Kreis-Schnittpunkten
        # Kreis um A mit Radius L1 und Kreis um B mit Radius L2 + L3
        d = AB
        if d > L1 + L2 + L3 or d < abs(L1 - L2 - L3):
            return []
        
        # Winkel zwischen A und B
        theta_AB = atan2(yB - yA, xB - xA)
        
        # Berechne mögliche Positionen des ersten Gelenks (P)
        # Nutze den Cosinus-Satz für Dreieck APB
        cos_alpha = (L1**2 + d**2 - (L2 + L3)**2) / (2 * L1 * d)
        if abs(cos_alpha) > 1:
            return []  # Keine reelle Lösung
        
        alpha = acos(cos_alpha)
        
        # Zwei mögliche Positionen für P (Spiegelungen)
        angles_P1 = theta_AB + alpha
        angles_P2 = theta_AB - alpha
        
        P1 = (xA + L1 * cos(angles_P1), yA + L1 * sin(angles_P1))
        P2 = (xA + L1 * cos(angles_P2), yA + L1 * sin(angles_P2))
        
        # Für jede Position von P berechne die Winkel
        for P in [P1, P2]:
            xP, yP = P
            # Vektor AP
            dx_AP = xP - xA
            dy_AP = yP - yA
            theta1 = atan2(dy_AP, dx_AP)
            
            # Vektor PB
            dx_PB = xB - xP
            dy_PB = yB - yP
            theta_PB = atan2(dy_PB, dx_PB)
            
            # Berechne Winkel zwischen L2 und L3
            # Nutze den Cosinus-Satz im Dreieck P-Q-B, wobei Q das zweite Gelenk ist
            PQ = L2
            QB = L3
            PB = sqrt(dx_PB**2 + dy_PB**2)
            
            if PB < tol:
                return []
            
            cos_beta = (PQ**2 + PB**2 - QB**2) / (2 * PQ * PB)
            if abs(cos_beta) > 1:
                return []
            
            beta = acos(cos_beta)
            
            # Zwei mögliche Winkel für theta2 (abhängig von der Orientierung)
            theta2_1 = theta_PB - beta
            theta2_2 = theta_PB + beta
            
            # Winkel theta2 ist der Winkel zwischen L1 und L2
            theta2_1_rel = theta2_1 - theta1
            theta2_2_rel = theta2_2 - theta1
            
            # Winkel theta3 ist der Winkel zwischen L2 und L3
            theta3_1 = theta_PB - theta2_1
            theta3_2 = theta_PB - theta2_2
            
            # Normalisiere Winkel auf [-pi, pi]
            theta2_1_rel = (theta2_1_rel + np.pi) % (2 * np.pi) - np.pi
            theta2_2_rel = (theta2_2_rel + np.pi) % (2 * np.pi) - np.pi
            theta3_1 = (theta3_1 + np.pi) % (2 * np.pi) - np.pi
            theta3_2 = (theta3_2 + np.pi) % (2 * np.pi) - np.pi
            
            solutions.append((theta1, theta2_1_rel, theta3_1))
            solutions.append((theta1, theta2_2_rel, theta3_2))
    
    return solutions

def plot_solution(A, B, L1, L2, L3, angles):
    """Plottet die Konfiguration der drei Linien."""
    theta1, theta2, theta3 = angles
    
    xA, yA = A
    xB, yB = B
    
    # Berechne die Position des ersten Gelenks (P)
    xP = xA + L1 * cos(theta1)
    yP = yA + L1 * sin(theta1)
    
    # Berechne die Position des zweiten Gelenks (Q)
    xQ = xP + L2 * cos(theta1 + theta2)
    yQ = yP + L2 * sin(theta1 + theta2)
    
    plt.figure()
    plt.plot([xA, xP], [yA, yP], 'r-', label=f'L1 = {L1}')
    plt.plot([xP, xQ], [yP, yQ], 'g-', label=f'L2 = {L2}')
    plt.plot([xQ, xB], [yQ, yB], 'b-', label=f'L3 = {L3}')
    plt.scatter([xA, xP, xQ, xB], [yA, yP, yQ, yB], c='k')
    plt.text(xA, yA, 'A', fontsize=12, ha='right')
    plt.text(xB, yB, 'B', fontsize=12, ha='left')
    plt.axis('equal')
    plt.legend()
    plt.title(f'Winkel: θ1={degrees(theta1):.1f}°, θ2={degrees(theta2):.1f}°, θ3={degrees(theta3):.1f}°')
    plt.grid(True)
    plt.show()

# Beispielaufruf
A = (0, 0)
B = (5, 0)
L1, L2, L3 = 3, 2, 2

solutions = calculate_angles(A, B, L1, L2, L3)
if not solutions:
    print("Keine Lösung möglich (Längen sind physikalisch nicht realisierbar).")
else:
    for i, sol in enumerate(solutions, 1):
        print(f"Lösung {i}: θ1 = {degrees(sol[0]):.2f}°, θ2 = {degrees(sol[1]):.2f}°, θ3 = {degrees(sol[2]):.2f}°")
        plot_solution(A, B, L1, L2, L3, sol)