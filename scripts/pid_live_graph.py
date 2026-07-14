import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button, Slider
import time
import matplotlib.ticker as ticker
import collections
import threading
import sys
import numpy as np

# ========================= CONFIG =========================
COM_PORT = 'COM7'
BAUD_RATE = 115200

PID_UPDATE_RATE = 200

MAX_POINTS = PID_UPDATE_RATE * 2      # 2 second
WINDOW_POINTS = PID_UPDATE_RATE * 4   # 4 seconds

# milliseconds per step (derived from update rate)
STEP_MS = 1000.0 / PID_UPDATE_RATE

MIN_RPM = 0
MAX_RPM = 3500

step_data = collections.deque(maxlen=WINDOW_POINTS)
rpm_data = collections.deque(maxlen=WINDOW_POINTS)
pwm_data = collections.deque(maxlen=WINDOW_POINTS)
current_data = collections.deque(maxlen=WINDOW_POINTS)

data_lock = threading.Lock()
new_data = False
sample_size = 0
rpm_state_lock = threading.Lock()
pending_target_rpm = None

# Updated default PID values
pid_p = 0.12
pid_i = 0.8
pid_d = 0.0
target_rpm = 250

ser = None
ser_lock = threading.Lock()
motor_running = False
motor_state_lock = threading.Lock()
pending_motor_running = None

def send_command(cmd):
    global ser
    with ser_lock:
        if ser and ser.is_open:
            try:
                ser.write((cmd + '\n').encode('ascii', 'ignore'))
                print(f"Sent: {cmd}")
            except Exception as e:
                print(f"Send error: {e}")

# ===================== SERIAL THREAD =====================
def read_serial():
    global new_data, ser, pending_target_rpm, pending_motor_running
    collecting = False
    collected_samples = 0

    try:
        with ser_lock:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {COM_PORT}")
        # Ensure motor is OFF by default on connect
        send_command("0")
        
        while True:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line == "START":
                    print("'START' received")
                    with motor_state_lock:
                        pending_motor_running = True
                    collecting = True
                    collected_samples = 0
                    with data_lock:
                        step_data.clear()
                        rpm_data.clear()
                        pwm_data.clear()
                        current_data.clear()

                elif line == "STOP":
                    print("'STOP' received")
                    with motor_state_lock:
                        pending_motor_running = False

                elif line.startswith("D,") and collecting:
                    try:
                        parts = line.split(',')
                        if len(parts) >= 4:
                            step = int(parts[1])
                            rpm = float(parts[2])
                            pwm = float(parts[3])
                            current_ma = float(parts[4]) if len(parts) >= 5 else np.nan

                            if step == 0:
                                collected_samples = 0
                                with data_lock:
                                    step_data.clear()
                                    rpm_data.clear()
                                    pwm_data.clear()
                                    current_data.clear()
                                print("Cleared on step 0")

                            with data_lock:
                                step_data.append(step)
                                rpm_data.append(rpm)
                                pwm_data.append(pwm)
                                current_data.append(current_ma)
                            collected_samples += 1
                            
                            new_data = True

                            if sample_size > 0 and collected_samples >= sample_size:
                                collecting = False
                    except ValueError:
                        continue

                elif line.startswith("R,"):
                    try:
                        rpm = int(line.split(',', 1)[1])
                        with rpm_state_lock:
                            pending_target_rpm = rpm
                        print(f"Received RPM: {rpm}")
                    except ValueError:
                        continue
            except Exception:
                time.sleep(0.05)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


# ====================== GUI ======================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
plt.subplots_adjust(bottom=0.38)

fig.suptitle('PID Controller Live Monitor & Tuner', fontsize=14)

# Main lines
line_rpm, = ax1.plot([], [], 'b-', label='RPM', linewidth=2)
line_pwm, = ax2.plot([], [], 'r-', label='PWM', linewidth=2)
line_current, = ax3.plot([], [], 'g-', label='Current (mA)', linewidth=2)

# Average lines
avg_rpm_line = ax1.axhline(y=0, color='blue', linestyle='--', alpha=0.7, label='Avg RPM')
avg_pwm_line = ax2.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Avg PWM')
avg_current_line = ax3.axhline(y=0, color='green', linestyle='--', alpha=0.7, label='Avg Current')

ax1.set_ylabel('RPM')
ax1.grid(True)
ax1.legend()

ax2.set_ylabel('PWM')
# X-axis: fixed 100ms grid (locator in step units)
# compute locator step in "steps" (axis units)
x_step = 100.0 / STEP_MS
locator = ticker.MultipleLocator(x_step)
ax1.xaxis.set_major_locator(locator)
ax2.xaxis.set_major_locator(locator)
ax3.xaxis.set_major_locator(locator)
# hide numeric tick labels on both axes; keep a single static xlabel on the bottom axis
ax1.tick_params(axis='x', labelbottom=False)
ax2.tick_params(axis='x', labelbottom=False)
ax3.tick_params(axis='x', labelbottom=False)
ax3.set_xlabel('Time (100ms grid)')
# Show PWM axis as percent labels (0%..100%) in 25% steps
ax2.set_yticks(np.linspace(0, 1800, 5))
ax2.set_yticklabels([f"{i}%" for i in range(0, 101, 25)])
ax2.grid(True)
ax2.legend()

ax3.set_ylabel('Current (mA)')
ax3.grid(True)
ax3.legend()

# PID Inputs
text_p   = TextBox(plt.axes([0.15, 0.28, 0.12, 0.05]), 'P', initial=str(pid_p))
text_i   = TextBox(plt.axes([0.35, 0.28, 0.12, 0.05]), 'I', initial=str(pid_i))
text_d   = TextBox(plt.axes([0.55, 0.28, 0.12, 0.05]), 'D', initial=str(pid_d))

def submit_pid(val):
    global pid_p, pid_i, pid_d
    try:
        pid_p = float(text_p.text)
        pid_i = float(text_i.text)
        pid_d = float(text_d.text)
        send_command(f"P={pid_p},{pid_i},{pid_d}")
    except:
        print("Invalid PID")

text_p.on_submit(submit_pid)
text_i.on_submit(submit_pid)
text_d.on_submit(submit_pid)

# RPM Slider
ax_slider = plt.axes([0.15, 0.20, 0.52, 0.04])
rpm_slider = Slider(ax_slider, 'Target RPM', MIN_RPM, MAX_RPM, valinit=target_rpm, valstep=1)

def update_rpm(val):
    global target_rpm
    target_rpm = int(val)
    send_command(f"R={target_rpm}")
    global new_data
    new_data = True

rpm_slider.on_changed(update_rpm)

# Sample count input
text_samples = TextBox(plt.axes([0.69, 0.14, 0.18, 0.04]), 'Samples', initial=str(sample_size))

def submit_samples(val):
    global sample_size
    try:
        sample_size = int(text_samples.text)
        if sample_size < 0:
            sample_size = 0
        print(f"Sample size → {sample_size} ({'unlimited' if sample_size == 0 else sample_size})")
    except ValueError:
        print("Invalid sample size")

text_samples.on_submit(submit_samples)

# Buttons
btn_toggle = Button(plt.axes([0.15, 0.12, 0.25, 0.06]), 'Start Motor', color='lightblue')
def toggle_motor(event=None):
    global motor_running, target_rpm
    try:
        if motor_running:
            # stop motor
            send_command("0")
            motor_running = False
            try:
                btn_toggle.label.set_text('Start Motor')
            except Exception:
                pass
        else:
            # send target rpm first, then start
            send_command(f"R={target_rpm}")
            time.sleep(0.02)
            send_command("1")
            motor_running = True
            try:
                btn_toggle.label.set_text('Stop Motor')
            except Exception:
                pass
    except Exception as e:
        print(f"Toggle error: {e}")

btn_toggle.on_clicked(toggle_motor)

btn_send_pid = Button(plt.axes([0.15, 0.04, 0.25, 0.06]), 'Send PID')
btn_send_rpm = Button(plt.axes([0.48, 0.04, 0.25, 0.06]), 'Send RPM')

btn_send_pid.on_clicked(lambda e: submit_pid(None))
btn_send_rpm.on_clicked(lambda e: update_rpm(rpm_slider.val))

# ===================== PLOT =====================
def init():
    ax1.set_xlim(0, MAX_POINTS - 1)
    ax2.set_xlim(0, MAX_POINTS - 1)
    ax3.set_xlim(0, MAX_POINTS - 1)
    ax1.set_ylim(0, 800)
    ax2.set_ylim(0, 1800)
    ax3.set_ylim(0, 3500)
    return line_rpm, line_pwm, line_current, avg_rpm_line, avg_pwm_line, avg_current_line

def update(frame):
    global new_data, target_rpm, pending_target_rpm, pending_motor_running, motor_running
    with data_lock:
        steps = list(step_data)
        rpms = list(rpm_data)
        pwms = list(pwm_data)
        currents = list(current_data)
        if new_data:
            new_data = False

    rpm_from_serial = None
    with rpm_state_lock:
        if pending_target_rpm is not None:
            rpm_from_serial = pending_target_rpm
            pending_target_rpm = None

    if rpm_from_serial is not None and int(rpm_slider.val) != rpm_from_serial:
        target_rpm = rpm_from_serial
        old_eventson = rpm_slider.eventson
        rpm_slider.eventson = False
        rpm_slider.set_val(rpm_from_serial)
        rpm_slider.eventson = old_eventson

    motor_state_from_serial = None
    with motor_state_lock:
        if pending_motor_running is not None:
            motor_state_from_serial = pending_motor_running
            pending_motor_running = None

    if motor_state_from_serial is not None:
        motor_running = motor_state_from_serial
        desired_label = 'Stop Motor' if motor_running else 'Start Motor'
        if btn_toggle.label.get_text() != desired_label:
            btn_toggle.label.set_text(desired_label)

    line_rpm.set_data(steps, rpms)
    line_pwm.set_data(steps, pwms)
    line_current.set_data(steps, currents)

    # Update average lines
    if rpms:
        avg_rpm = np.mean(rpms[-100:])
        avg_rpm_line.set_ydata([avg_rpm])
    if pwms:
        avg_pwm = np.mean(pwms)
        avg_pwm_line.set_ydata([avg_pwm])
    valid_currents = [c for c in currents if not np.isnan(c)]
    if valid_currents:
        avg_current = np.mean(valid_currents)
        avg_current_line.set_ydata([avg_current])

    if steps:
        max_x = max(steps)
        if sample_size == 0:
            ax1.set_xlim(max(0, max_x - WINDOW_POINTS + 1), max_x)
            ax2.set_xlim(max(0, max_x - WINDOW_POINTS + 1), max_x)
            ax3.set_xlim(max(0, max_x - WINDOW_POINTS + 1), max_x)
        else:
            ax1.set_xlim(0, max_x)
            ax2.set_xlim(0, max_x)
            ax3.set_xlim(0, max_x)

    # Scale RPM axis to 125% of the maximum visible RPM
    if rpms:
        max_rpm = max(rpms)
        ax1.set_ylim(0, max(100, max_rpm * 1.25))
    else:
        ax1.set_ylim(0, 800)
    ax2.set_ylim(0, 1800)
    if valid_currents:
        max_current = max(valid_currents)
        ax3.set_ylim(0, max(100, max_current * 1.25))
    else:
        ax3.set_ylim(0, 3500)

    return line_rpm, line_pwm, line_current, avg_rpm_line, avg_pwm_line, avg_current_line


# ===================== START =====================
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

ani = FuncAnimation(fig, update, init_func=init, interval=80, blit=False)

plt.tight_layout(rect=[0, 0.34, 1, 0.95])
print("GUI started. Waiting for data...")
try:
    plt.show()
finally:
    print("Exiting: sending 0 to serial to stop motor")
    try:
        send_command("0")
    except Exception as e:
        print(f"Failed to send stop command: {e}")
    time.sleep(0.1)
    with ser_lock:
        try:
            if ser and ser.is_open:
                ser.close()
                print("Serial port closed")
        except Exception as e:
            print(f"Error closing serial: {e}")