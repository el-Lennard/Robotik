import pandas as pd
import matplotlib.pyplot as plt

# Datei einlesen (Leerzeichen als Trennzeichen)
file_path = "C:/Daten/Master/2.Semester/Robotik/Praktikum/Robotik_Library_LKEW/RTDE_Python_Client_Library/examples/robot_data_Aufgabe2.csv"
df = pd.read_csv(file_path, delim_whitespace=True)

# Parameter definieren
params = ['target_q_', 'target_qd_', 'target_qdd_', 'actual_q_', 'actual_qd_']
columns = ['timestamp'] + [f"{p}{i}" for p in params for i in range(6)]

# Nur relevante Spalten behalten
df_filtered = df[columns]

# Plotten
for param in params:
    plt.figure(figsize=(10, 4))
    for i in range(6):
        joint_col = f"{param}{i}"
        plt.plot(df_filtered['timestamp'], df_filtered[joint_col], label=joint_col)
    plt.title(f"{param.strip('_')} über Zeit")
    plt.xlabel("Zeit [s]")
    plt.ylabel("Wert")
    plt.legend()
    plt.tight_layout()
    plt.grid(True)
    plt.show()