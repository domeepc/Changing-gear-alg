#
# PYTHON SCRIPT FOR PREDICTING RPM FOR GEAR SHIFTING IN  RACING MOTORCYCLE
# 

import can
import time
import subprocess

# ------------------------
# Constants
# ------------------------

UPSHIFT_THRESHOLD = 13500
DOWNSHIFT_THRESHOLD = 2000
CAN_ID_FRAME = 0x123    
SAMPLE_TIME = 0.05      # Every 50ms it takes rpm and calculates the derivatives of said rpm value
PREDICTION_TIME = 0.3   # Prediction time aka how far into the future should the program predict
                        # (driver reaction time + mechanical delay = 300ms)

# EMA smoothing factors
alpha_rpm = 0.2
alpha_dot = 0.4
alpha_ddot = 0.3

# ------------------------
# State variables
# ------------------------
rpm_filtered = 0.0
rpm_dot_filtered = 0.0
rpm_ddot_filtered = 0.0

last_rpm = 0.0
last_rpm_dot = 0.0
last_tick = 0.0

current_rpm = 0.0

sound_playing = False
sound_cooldown_until = 0.0

first_run = 0

# ------------------------
# Functions
# ------------------------
def get_current_rpm(frame):
    """Extract RPM from a CAN frame."""
    if frame.arbitration_id == CAN_ID_FRAME:
        if frame.dlc >= 2:
            rpm = (frame.data[0] << 8) | frame.data[1]
            return rpm
    return None  # frame not valid

def calculate_derivatives(rpm):
    """Update filtered RPM, first derivative, and second derivative."""
    global rpm_filtered, rpm_dot_filtered, rpm_ddot_filtered
    global last_rpm, last_rpm_dot, first_run

    if first_run == 0 and rpm > 0:
        rpm_filtered = rpm
        last_rpm = rpm
        first_run = 1

    # 1. Exponential smoothing of RPM
    rpm_filtered = alpha_rpm * rpm + (1 - alpha_rpm) * rpm_filtered

    # 2. First derivative
    rpm_dot = (rpm_filtered - last_rpm) / SAMPLE_TIME
    rpm_dot_filtered = alpha_dot * rpm_dot + (1 - alpha_dot) * rpm_dot_filtered

    # 3. Second derivative
    rpm_ddot = (rpm_dot_filtered - last_rpm_dot) / SAMPLE_TIME
    rpm_ddot_filtered = alpha_ddot * rpm_ddot + (1 - alpha_ddot) * rpm_ddot_filtered

    # 4. Update last values
    last_rpm = rpm_filtered
    last_rpm_dot = rpm_dot_filtered

def predict_rpm():
    """Predict RPM using 2nd order Taylor expansion."""
    return rpm_filtered + rpm_dot_filtered * PREDICTION_TIME + 0.5 * rpm_ddot_filtered * (PREDICTION_TIME ** 2)

def PlayUpShiftSound():
    subprocess.Popen(
        ["sox", "-n", "-d", "synth", "0.1", "sine", "800"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def PlayDownShiftSound():
    subprocess.Popen(
        ["sox", "-n", "-d", "synth", "0.1", "sine", "500"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

# ------------------------
# Setup CAN bus
# ------------------------
bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
print("Listening for CAN messages...")

# ------------------------
# Main loop
# ------------------------
predicted_rpm = 0.0


try:

    upshift_alerted = False
    downshift_alerted = False
    while True:
        msg = bus.recv(timeout=0)
        if msg:
            current_rpm = get_current_rpm(msg)

        now = time.time()

        if now - last_tick >= SAMPLE_TIME:
            last_tick = now
            if current_rpm is not None:
                calculate_derivatives(current_rpm)
                predicted_rpm = predict_rpm()
                print(f"Current RPM: {current_rpm}, Predicted RPM: {predicted_rpm:.1f}")

                if predicted_rpm >= UPSHIFT_THRESHOLD and rpm_dot_filtered > 0:
                    if not upshift_alerted:
                        print("Upshift now!!")
                        PlayUpShiftSound()
                        upshift_alerted = True 
                else:
                    upshift_alerted = False
                if predicted_rpm <= DOWNSHIFT_THRESHOLD and rpm_dot_filtered < 0:
                    if not downshift_alerted:
                        print("Downshift now!!")
                        PlayDownShiftSound()
                        downshift_alerted = True
                else:
                    downshift_alerted = False

except KeyboardInterrupt:
    print("\nExiting gracefully.")
