import serial
import time
import json
import numpy as np

class RoArmM1:
    def __init__(self, port='COM4', baudrate=115200, timeout=2):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Warte, bis Verbindung steht
        
        # Anfangswerte für alle 5 Servos
        self.servo_positions = [180, 0, 90, 90, 180]  # P1, P2, P3, P4, P5
        self.servo_speeds = [200, 200, 200, 200, 200]
        self.servo_accelerations = [60, 60, 60, 60, 60]

    def move_servos(self):
        """Bewegt alle Servos basierend auf den aktuellen Werten."""
        # Erstelle das JSON-kompatible Dictionary
        command = {
            "T": 1,
            "P1": self.servo_positions[0],
            "P2": self.servo_positions[1],
            "P3": self.servo_positions[2],
            "P4": self.servo_positions[3],
            "P5": self.servo_positions[4],
            "S1": self.servo_speeds[0],
            "S2": self.servo_speeds[1],
            "S3": self.servo_speeds[2],
            "S4": self.servo_speeds[3],
            "S5": self.servo_speeds[4],
            "A1": self.servo_accelerations[0],
            "A2": self.servo_accelerations[1],
            "A3": self.servo_accelerations[2],
            "A4": self.servo_accelerations[3],
            "A5": self.servo_accelerations[4]
        }
        
        # Wandelt das Dictionary in einen JSON-String um
        command_str = json.dumps(command) + '\r\n'
        
        # Sendet den Befehl an den Roboterarm
        self.ser.write(command_str.encode())
        print(f"Gesendet: {command_str}")

    def update_servo(self, servo_id, angle):
        """Aktualisiert den Winkel eines bestimmten Servos."""
        if not (1 <= servo_id <= 5 and 0 <= angle <= 180):
            print("Ungültige Werte!")
            return
        self.servo_positions[servo_id - 1] = angle

    def T_2_rotvec(T):
        """
        homogeneous trafo matrix to rotation vector representation - Kinematik Grundlagen S.13
        """
        x, y, z = T[0:3, 3] # Verschiebungsvektor
        R = T[0:4, 0:4]     # Rotationsmatrix
        theta = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
        K = 1/(2*np.sin(theta))*np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        pose = np.array([x, y, z, theta*K[0], theta*K[1], theta*K[2]])
        return pose

    def fk_ur(dh_para, q):
        """
        Forward Kinematics for UR type robots - Vorwärtskinematik S.17
        """
        T_0_6 = np.eye(4)
        i = 0
        for lines in dh_para:
            T_0_6 = T_0_6 @ dh(lines[0], lines[1], lines[2], q[i])
            i = i + 1
        return T_0_6

    def close(self):
        self.ser.close()
        print("Verbindung geschlossen.")


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic) - Vorwärtskinematik S.16
    """
    ct, st = np.cos(theta), np.sin(theta)
    calph, salph = np.cos(alpha), np.sin(alpha)
    T = np.array([[ct,  -st*calph,  st*salph,   a*ct],
                [st,  ct*calph,   -ct*salph,  a*st],
                [0,   salph,      calph,      d],
                [0,   0,          0,          1]])
    return T

# Beispielnutzung
if __name__ == "__main__":
    arm = RoArmM1(port='COM4')  # COM-Port angepasst
    
    a = [0.00000, -0.041,0, -0.21325, 0.00000, 0.00000, 0.0000]
    d = [0.11574, 0.00000, 0.00000, 0.0, 0.0, 0.10854]
    alpha = [0, -1.570796327, 0, 0, 0]
    q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
    joint_direction = [1, 1, -1, 1, 1, 1]

    ur3 = np.array([alpha, a, d, q_home_offset]).T # Vorwärtskinematik S.17
    print('ur3:\n', ur3)
    print('----------------------------------------\n')

    q = np.array([0, -90, -30, 40, 60, 0]) / 180 * np.pi

    T_0_6 = arm.fk_ur(ur3, q)
    # print(T_0_6)
    print('T_2_rotvec(T_0_6):\n', arm.T_2_rotvec(T_0_6)) # T_2_rotvec: homogeneous trafo matrix to rotation vector representation
    print('----------------------------------------\n')

    try:
        while True:
            print("\nGib die Servo-ID (1-5) ein, oder 'q' zum Beenden:")
            servo_id_input = input("Servo-ID: ")
            if servo_id_input.lower() == 'q':
                break
            
            # Überprüfen, ob die Eingabe eine gültige Zahl ist
            if not servo_id_input.isdigit():
                print("Ungültige Eingabe! Bitte eine Zahl eingeben.")
                continue
            
            servo_id = int(servo_id_input)
            if not (1 <= servo_id <= 5):
                print("Servo-ID muss zwischen 1 und 5 liegen!")
                continue
            
            # Eingabe für den Winkel des Servos
            angle_input = input(f"Gib den Winkel für Servo {servo_id} (0-180) ein: ")
            if not angle_input.isdigit():
                print("Ungültige Eingabe! Bitte eine Zahl eingeben.")
                continue
            
            angle = int(angle_input)
            if not (0 <= angle <= 180):
                print("Der Winkel muss zwischen 0 und 180 liegen!")
                continue
            
            # Bewege das Servo
            arm.update_servo(servo_id, angle)
            arm.move_servos()
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nProgramm abgebrochen.")
    
    finally:
        # Verbindung schließen
        arm.close()
