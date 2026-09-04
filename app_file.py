import dearpygui.dearpygui as dpg
import time
from collections import deque
import json
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import websocket
import subprocess
import threading
import serial
import serial.tools.list_ports
#connection 
ESP_IP = "192.168.29.78"       
WS_PORT = 81
WS_URL = f"ws://{ESP_IP}:{WS_PORT}/"

BAUD_RATE = 115200

# Connection modes
MODE_WEBSOCKET = "WebSocket"
MODE_USB = "USB"

connection_mode = MODE_WEBSOCKET
is_connected = False

ws = None
ws_lock = threading.Lock()

serial_conn = None
serial_lock = threading.Lock()

# USB sensor health 
last_sensor_data_time = 0.0
sensor_error = None
USB_DATA_TIMEOUT = 2.0
USB_RECONNECT_DELAY = 1.0
usb_candidate_port = None
usb_prompt_shown = False
usb_declined_port = None

start_time = time.time()
last_graph_update = 0
notification_start_time = 0

#Magnetic field data
selected_unit = "G"

UNIT_FACTORS = {
    "G": 1.0,
    "mG": 1000.0,
    "uT": 100.0,
    "mT": 0.1,
}
Bx = 0.0
By = 0.0
Bz = 0.0

B_magnitude = 0.0

# Magnetic field controller state
# The Arduino PID controls Bz. The internal setpoint is stored in Gauss.
controller_setpoint_g = 0.0
pid_running = False
last_reported_error = None

# Calibration bias
Bx_bias = 0.0
By_bias = 0.0
Bz_bias = 0.0

calibrating = False
calibration_samples = []
CALIBRATION_COUNT = 100

# Calibration settings
CALIBRATION_METHOD = "Mean"
CALIBRATION_METHODS = ["Mean", "Median", "Trimmed Mean"]
CALIBRATION_TRIM_PERCENT = 10

logged_data = []
is_logging = False
last_logged_time = 0

def process_sensor_message(message):
    global Bx, By, Bz, B_magnitude
    global last_sensor_data_time, sensor_error

    try:
        if isinstance(message, bytes):
            message = message.decode("utf-8", errors="ignore")

        message = message.strip()

        if not message:
            return False

        payload = json.loads(message)

        # Arduino may report recoverable RM3100/I2C errors.
        if "error" in payload:
            sensor_error = str(payload["error"])
            return False

        # Controller status packets are informational, not sensor packets.
        if "status" in payload:
            return False

        if not all(key in payload for key in ("Bx", "By", "Bz")):
            sensor_error = "INCOMPLETE_DATA"
            return False

        Bx = float(payload["Bx"])
        By = float(payload["By"])
        Bz = float(payload["Bz"])

        B_magnitude = (Bx**2 + By**2 + Bz**2) ** 0.5

        last_sensor_data_time = time.time()
        sensor_error = None
        return True

    except (json.JSONDecodeError, TypeError, ValueError):
        # Ignore Arduino startup text such as "RM3100 Measurement".
        return False


#callibration

def start_calibration():
    global calibrating, calibration_samples

    if not is_connected:
        show_notification("Connect sensor before calibration")
        return

    calibration_samples = []
    calibrating = True

    dpg.set_item_label("calibrate_button", "CALIBRATING...")
    dpg.disable_item("calibrate_button")
    show_notification(f"Calibrating: collect {CALIBRATION_COUNT} samples")


def finish_calibration():
    global Bx_bias, By_bias, Bz_bias
    global calibrating, calibration_samples

    if not calibration_samples:
        calibrating = False
        dpg.enable_item("calibrate_button")
        dpg.set_item_label("calibrate_button", "CALIBRATE ZERO")
        show_notification("Calibration failed: no samples")
        return

    xs = [s[0] for s in calibration_samples]
    ys = [s[1] for s in calibration_samples]
    zs = [s[2] for s in calibration_samples]

    def calculate(values):
        if CALIBRATION_METHOD == "Median":
            values = sorted(values)
            n = len(values)
            mid = n // 2
            if n % 2:
                return values[mid]
            return (values[mid - 1] + values[mid]) / 2.0

        if CALIBRATION_METHOD == "Trimmed Mean":
            values = sorted(values)
            trim = int(len(values) * CALIBRATION_TRIM_PERCENT / 100)

            # Always retain at least one sample.
            if 2 * trim >= len(values):
                trim = 0

            trimmed = values[trim:len(values) - trim]

            return sum(trimmed) / len(trimmed)

        # Default: arithmetic mean
        return sum(values) / len(values)

    Bx_bias = calculate(xs)
    By_bias = calculate(ys)
    Bz_bias = calculate(zs)

    calibration_samples = []
    calibrating = False

    dpg.enable_item("calibrate_button")
    dpg.set_item_label("calibrate_button", "CALIBRATE ZERO")

    dpg.set_value(
        "bias_text",
        f"Bias: Bx={Bx_bias:.4f} G, By={By_bias:.4f} G, Bz={Bz_bias:.4f} G"
    )

    show_notification(
        f"Calibration complete ({CALIBRATION_METHOD}, {CALIBRATION_COUNT} samples)"
    )


def save_calibration_settings():
    global CALIBRATION_COUNT, CALIBRATION_METHOD

    try:
        samples = int(dpg.get_value("cal_samples_input"))

        if samples < 10:
            show_notification("Use at least 10 calibration samples")
            return

        if samples > 10000:
            show_notification("Maximum calibration samples: 10000")
            return

        CALIBRATION_COUNT = samples
        CALIBRATION_METHOD = dpg.get_value("cal_method_combo")

        dpg.configure_item("settings_window", show=False)

        show_notification(
            f"Calibration settings saved: {CALIBRATION_COUNT} samples, "
            f"{CALIBRATION_METHOD}"
        )

    except (TypeError, ValueError):
        show_notification("Invalid calibration settings")


def show_settings_window():
    if dpg.does_item_exist("settings_window"):
        dpg.set_value("popup_esp_ip_input", ESP_IP)
        dpg.set_value("popup_ws_port_input", WS_PORT)
        dpg.set_value("cal_samples_input", CALIBRATION_COUNT)
        dpg.set_value("cal_method_combo", CALIBRATION_METHOD)
        dpg.configure_item("settings_window", show=True)
        return

    vp_w = dpg.get_viewport_width()
    vp_h = dpg.get_viewport_height()

    with dpg.window(
        label="Settings",
        modal=True,
        show=True,
        tag="settings_window",
        width=480,
        height=430,
        pos=[(vp_w - 480) // 2, (vp_h - 430) // 2],
        no_resize=True
    ):

        with dpg.tab_bar(tag="settings_tabs"):

            #connection tab

            with dpg.tab(label="Connection"):

                dpg.add_spacer(height=8)

                dpg.add_text(
                    "WebSocket Connection Settings",
                    color=[100, 220, 255, 255]
                )

                dpg.add_separator()
                dpg.add_spacer(height=12)

                dpg.add_text("ESP8266 IP:")

                dpg.add_input_text(
                    default_value=ESP_IP,
                    tag="popup_esp_ip_input",
                    width=280
                )

                dpg.add_spacer(height=8)

                dpg.add_text("WebSocket Port:")

                dpg.add_input_int(
                    default_value=WS_PORT,
                    tag="popup_ws_port_input",
                    width=150
                )

                dpg.add_spacer(height=20)

                dpg.add_button(
                    label="Connect",
                    callback=apply_connection,
                    width=120
                )

            ##callibration tab

            with dpg.tab(label="Calibration"):

                dpg.add_spacer(height=8)

                dpg.add_text(
                    "Zero Calibration Settings",
                    color=[100, 220, 255, 255]
                )

                dpg.add_separator()
                dpg.add_spacer(height=12)

                dpg.add_text("Number of samples")

                dpg.add_input_int(
                    default_value=CALIBRATION_COUNT,
                    min_value=10,
                    max_value=10000,
                    min_clamped=True,
                    max_clamped=True,
                    width=180,
                    tag="cal_samples_input"
                )

                dpg.add_spacer(height=12)

                dpg.add_text("Calibration method")

                dpg.add_combo(
                    items=CALIBRATION_METHODS,
                    default_value=CALIBRATION_METHOD,
                    width=180,
                    tag="cal_method_combo"
                )

                dpg.add_spacer(height=10)

                dpg.add_text(
                    "Mean: average of all samples",
                    color=[180, 180, 180, 255]
                )

                dpg.add_text(
                    "Median: robust against isolated spikes",
                    color=[180, 180, 180, 255]
                )

                dpg.add_text(
                    "Trimmed Mean: removes the highest and lowest 10%",
                    color=[180, 180, 180, 255]
                )

                dpg.add_spacer(height=18)

                dpg.add_button(
                    label="Save Calibration Settings",
                    width=210,
                    callback=save_calibration_settings
                )

        dpg.add_spacer(height=15)

        dpg.add_button(
            label="Close",
            width=100,
            callback=lambda: dpg.configure_item(
                "settings_window",
                show=False
            )
        )

#Unit conversion

def convert_from_gauss(value):
    return value * UNIT_FACTORS[selected_unit]

def change_layout(sender, app_data):
    if app_data == "All":

        # Show all containers
        dpg.configure_item("bx_container", show=True, width=450, height=275)
        dpg.configure_item("by_container", show=True, width=450, height=275)
        dpg.configure_item("bz_container", show=True, width=450, height=275)
        dpg.configure_item("bmag_container", show=True, width=450, height=275)

        # Restore plot sizes
        dpg.configure_item("bx_plot", width=-1, height=245)
        dpg.configure_item("by_plot", width=-1, height=245)
        dpg.configure_item("bz_plot", width=-1, height=245)
        dpg.configure_item("bmag_plot", width=-1, height=245)

    elif app_data == "Bx":

        dpg.configure_item("bx_container", show=True, width=-1, height=570)
        dpg.configure_item("by_container", show=False)
        dpg.configure_item("bz_container", show=False)
        dpg.configure_item("bmag_container", show=False)

        dpg.configure_item("bx_plot", width=-1, height=535)
    elif app_data == "By":

        dpg.configure_item("bx_container", show=False)
        dpg.configure_item("by_container", show=True, width=-1, height=570)
        dpg.configure_item("bz_container", show=False)
        dpg.configure_item("bmag_container", show=False)

        dpg.configure_item("by_plot", width=-1, height=535)


    elif app_data == "Bz":

        dpg.configure_item("bx_container", show=False)
        dpg.configure_item("by_container", show=False)
        dpg.configure_item("bz_container", show=True, width=-1, height=570)
        dpg.configure_item("bmag_container", show=False)

        dpg.configure_item("bz_plot", width=-1, height=535)

    elif app_data == "Mag":

        dpg.configure_item("bx_container", show=False)
        dpg.configure_item("by_container", show=False)
        dpg.configure_item("bz_container", show=False)
        dpg.configure_item("bmag_container", show=True, width=-1, height=570)

        dpg.configure_item("bmag_plot", width=-1, height=535)
def change_unit(sender, app_data):
    global selected_unit

    selected_unit = app_data

    factor = UNIT_FACTORS[selected_unit]

    # Keep the controller setpoint physically unchanged when display units change.
    if dpg.does_item_exist("setpoint_input"):
        dpg.set_value("setpoint_input", controller_setpoint_g * factor)

    if dpg.does_item_exist("setpoint_status"):
        dpg.set_value(
            "setpoint_status",
            f"Target: {controller_setpoint_g * factor:.4f} {selected_unit}"
        )

    # Top values
    dpg.set_value(
        "bx_text",
        f"Bx : {Bx * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "by_text",
        f"By : {By * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "bz_text",
        f"Bz : {Bz * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "bmag_text",
        f"|B| : {B_magnitude * factor:.4f} {selected_unit}"
    )

    # Graph Y-axis labels
    dpg.configure_item(
        "bx_y_axis",
        label=f"Bx ({selected_unit})"
    )

    dpg.configure_item(
        "by_y_axis",
        label=f"By ({selected_unit})"
    )

    dpg.configure_item(
        "bz_y_axis",
        label=f"Bz ({selected_unit})"
    )

    dpg.configure_item(
        "bmag_y_axis",
        label=f"|B| ({selected_unit})"
    )


def update_unit_display():
    factor = UNIT_FACTORS[selected_unit]

    dpg.set_value("units_text", f"{selected_unit}")

    dpg.set_value("bx_text", f"Bx : {Bx * factor:.4f} {selected_unit}")
    dpg.set_value("by_text", f"By : {By * factor:.4f} {selected_unit}")
    dpg.set_value("bz_text", f"Bz : {Bz * factor:.4f} {selected_unit}")
    dpg.set_value("bmag_text",
                  f"|B| : {B_magnitude * factor:.4f} {selected_unit}")

def on_ws_message(websocket_app, message):
    # Ignore WebSocket data while USB is the active mode
    if connection_mode != MODE_WEBSOCKET:
        return

    process_sensor_message(message)


def on_ws_open(websocket_app):
    global is_connected, ws

    if connection_mode != MODE_WEBSOCKET:
        try:
            websocket_app.close()
        except Exception:
            pass
        return

    with ws_lock:
        ws = websocket_app

    is_connected = True


def on_ws_close(websocket_app, close_status_code, close_msg):
    global is_connected, ws

    if connection_mode == MODE_WEBSOCKET:
        is_connected = False

    with ws_lock:
        if ws is websocket_app:
            ws = None


def on_ws_error(websocket_app, error):
    global is_connected

    if connection_mode == MODE_WEBSOCKET:
        is_connected = False


def websocket_worker():
    global is_connected

    while True:

        if connection_mode != MODE_WEBSOCKET:
            time.sleep(0.5)
            continue

        try:
            app = websocket.WebSocketApp(
                WS_URL,
                on_open=on_ws_open,
                on_message=on_ws_message,
                on_error=on_ws_error,
                on_close=on_ws_close
            )

            app.run_forever(
                ping_interval=20,
                ping_timeout=10
            )

        except Exception:
            is_connected = False

        if connection_mode == MODE_WEBSOCKET:
            time.sleep(3)


def retry_connection():
    global ws

    with ws_lock:
        active_ws = ws

    if active_ws is not None:
        try:
            active_ws.close()
        except Exception:
            pass

    show_notification("Reconnecting...")


#USB Serial

def find_arduino_port():

    ports = serial.tools.list_ports.comports()

    for port in ports:
        text = (
            f"{port.device} "
            f"{port.description or ''} "
            f"{port.manufacturer or ''} "
            f"{port.hwid or ''}"
        ).upper()

        # Ignore Bluetooth virtual serial ports
        if "BLUETOOTH" in text:
            continue

        keywords = [
            "ARDUINO",
            "MEGA",
            "CH340",
            "CH341",
            "USB-SERIAL",
            "CP210",
            "FTDI",
            "USB UART",
        ]

        if any(keyword in text for keyword in keywords):
            return port.device

    return None


def usb_reader(port):
    global serial_conn, is_connected
    global last_sensor_data_time, sensor_error

    while connection_mode == MODE_USB:
        ser = None

        try:
            ser = serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.2
            )

            # Arduino Mega resets when the serial port opens.
            time.sleep(2)

            with serial_lock:
                serial_conn = ser

            is_connected = False
            last_sensor_data_time = time.time()
            sensor_error = None

            while connection_mode == MODE_USB:
                try:
                    line = ser.readline()

                    if line:
                        valid = process_sensor_message(line)
                        if valid:
                            is_connected = True

                    # If no valid RM3100 packet arrives for too long,
                    # close the port and reconnect.
                    if (
                        is_connected
                        and time.time() - last_sensor_data_time > USB_DATA_TIMEOUT
                    ):
                        sensor_error = "NO_DATA"
                        is_connected = False
                        break

                except (serial.SerialException, OSError):
                    sensor_error = "SERIAL_ERROR"
                    is_connected = False
                    break

        except (serial.SerialException, OSError):
            sensor_error = "PORT_OPEN_ERROR"
            is_connected = False

        except Exception:
            sensor_error = "USB_READER_ERROR"
            is_connected = False

        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

            with serial_lock:
                if serial_conn is ser:
                    serial_conn = None

        if connection_mode == MODE_USB:
            time.sleep(USB_RECONNECT_DELAY)

    is_connected = False


def switch_to_usb():
    global connection_mode, is_connected, usb_candidate_port
    global usb_prompt_shown, usb_declined_port, sensor_error

    port = usb_candidate_port

    if not port:
        return

    # Stop WebSocket
    with ws_lock:
        active_ws = ws

    if active_ws is not None:
        try:
            active_ws.close()
        except Exception:
            pass

    connection_mode = MODE_USB
    is_connected = False
    sensor_error = None
    usb_prompt_shown = False
    usb_declined_port = None

    show_notification(f"Switching to USB ({port})...")

    threading.Thread(
        target=usb_reader,
        args=(port,),
        daemon=True
    ).start()


def keep_websocket():
    global usb_prompt_shown, usb_declined_port

    usb_declined_port = usb_candidate_port
    usb_prompt_shown = False

    show_notification("Staying in WebSocket mode")


def usb_monitor():
    global usb_candidate_port, usb_prompt_shown
    global usb_declined_port, connection_mode, is_connected

    previous_port = None

    while True:

        current_port = find_arduino_port()

        # Arduino inserted
        if current_port is not None and previous_port is None:

            usb_candidate_port = current_port

            if connection_mode == MODE_WEBSOCKET:
                if current_port != usb_declined_port:
                    usb_prompt_shown = False

        # Arduino removed
        if current_port is None and previous_port is not None:

            if connection_mode == MODE_USB:

                connection_mode = MODE_WEBSOCKET
                is_connected = False
                sensor_error = None

                with serial_lock:
                    if serial_conn is not None:
                        try:
                            serial_conn.close()
                        except Exception:
                            pass

                show_notification("Arduino removed - returning to WebSocket")

            usb_candidate_port = None
            usb_prompt_shown = False
            usb_declined_port = None

        previous_port = current_port

        time.sleep(1)

#connection Manager

def apply_connection():
    global ESP_IP, WS_PORT, WS_URL

    ESP_IP = dpg.get_value("popup_esp_ip_input")
    WS_PORT = int(dpg.get_value("popup_ws_port_input"))
    WS_URL = f"ws://{ESP_IP}:{WS_PORT}/"

    if dpg.does_item_exist("settings_window"):
        dpg.configure_item("settings_window", show=False)

    retry_connection()


def show_networks_popup():
    show_settings_window()


#USB prompt

def show_usb_prompt():
    global usb_prompt_shown

    if usb_candidate_port is None:
        return

    if usb_prompt_shown:
        return

    usb_prompt_shown = True

    if dpg.does_item_exist("usb_prompt"):
        dpg.configure_item("usb_prompt", show=True)
        dpg.set_value(
            "usb_prompt_text",
            f"Arduino detected on {usb_candidate_port}."
        )
        return

    vp_w = dpg.get_viewport_width()
    vp_h = dpg.get_viewport_height()

    with dpg.window(
        label="Arduino Detected",
        modal=True,
        show=True,
        tag="usb_prompt",
        width=440,
        height=190,
        pos=[(vp_w - 440) // 2, (vp_h - 190) // 2],
        no_resize=True
    ):

        dpg.add_text(
            "Arduino detected",
            color=[100, 220, 255, 255]
        )

        dpg.add_spacer(height=10)

        dpg.add_text(
            "",
            tag="usb_prompt_text"
        )

        dpg.add_text(
            "Do you want to switch from WebSocket mode "
            "to direct USB mode?"
        )

        dpg.add_spacer(height=15)

        with dpg.group(horizontal=True):

            dpg.add_button(
                label="Switch to USB",
                callback=lambda: (
                    dpg.configure_item("usb_prompt", show=False),
                    switch_to_usb()
                ),
                width=140
            )

            dpg.add_spacer(width=15)

            dpg.add_button(
                label="Keep WebSocket",
                callback=lambda: (
                    dpg.configure_item("usb_prompt", show=False),
                    keep_websocket()
                ),
                width=140
            )

    dpg.set_value(
        "usb_prompt_text",
        f"Arduino detected on {usb_candidate_port}."
    )


# ============================================================================
# MAGNETIC FIELD CONTROLLER COMMUNICATION
# ============================================================================

def send_controller_command(command):
    # Direct USB communication is implemented now.
    # WebSocket control can be added later when the ESP8266 forwards commands.
    if connection_mode != MODE_USB:
        show_error("Controller commands currently require USB connection")
        return False

    with serial_lock:
        ser = serial_conn

    if ser is None or not ser.is_open:
        show_error("Arduino serial connection is not available")
        return False

    try:
        with serial_lock:
            ser.write((command + "\n").encode("utf-8"))
            ser.flush()
        return True
    except (serial.SerialException, OSError) as error:
        show_error(f"Command failed: {error}")
        return False


def apply_setpoint(sender=None, app_data=None, user_data=None):
    global controller_setpoint_g

    try:
        value_in_display_units = float(dpg.get_value("setpoint_input"))
    except (TypeError, ValueError):
        show_error("Invalid setpoint")
        return

    controller_setpoint_g = value_in_display_units / UNIT_FACTORS[selected_unit]

    if send_controller_command(f"SET {controller_setpoint_g:.6f}"):
        dpg.set_value(
            "setpoint_status",
            f"Target: {value_in_display_units:.4f} {selected_unit}"
        )
        show_notification("Setpoint sent")


def start_pid(sender=None, app_data=None, user_data=None):
    global controller_setpoint_g, pid_running

    try:
        value_in_display_units = float(dpg.get_value("setpoint_input"))
    except (TypeError, ValueError):
        show_error("Invalid setpoint")
        return

    controller_setpoint_g = value_in_display_units / UNIT_FACTORS[selected_unit]

    if not send_controller_command(f"SET {controller_setpoint_g:.6f}"):
        return

    if send_controller_command("START"):
        pid_running = True
        dpg.set_value("pid_status", "PID: RUNNING")
        show_notification("PID started")


def stop_pid(sender=None, app_data=None, user_data=None):
    global pid_running

    if send_controller_command("STOP"):
        pid_running = False
        dpg.set_value("pid_status", "PID: STOPPED")
        show_notification("PID stopped")


# Notification / error toast

def show_toast(message, is_error=False):

    global notification_start_time

    if not dpg.does_item_exist("notification_text"):
        return

    dpg.set_value("notification_text", str(message))
    dpg.configure_item(
        "notification_text",
        color=[255, 100, 100, 255] if is_error else [220, 220, 220, 255]
    )

    # Small notification in the top-right corner.
    vp_w = dpg.get_viewport_width()
    popup_width = 330
    margin = 20

    dpg.configure_item(
        "notification_popup",
        pos=[max(margin, vp_w - popup_width - margin), margin]
    )

    dpg.show_item("notification_popup")
    notification_start_time = time.time()


def show_notification(message):
    show_toast(message, is_error=False)


def show_error(message):
    show_toast(f"ERROR: {message}", is_error=True)

#Excel logging

def export_excel():

    root = tk.Tk()
    root.withdraw()

    file_path = filedialog.asksaveasfilename(
        defaultextension=".xlsx",
        filetypes=[
            ("Excel files", "*.xlsx")
        ],
        title="Save Magnetic Field Data"
    )

    root.destroy()

    if not file_path:
        return

    df = pd.DataFrame(logged_data)

    df.to_excel(
        file_path,
        index=False
    )

    show_notification("Excel Exported")


def toggle_logging():

    global is_logging

    is_logging = not is_logging

    if is_logging:

        dpg.set_item_label(
            "logging_button",
            "STOP LOGGING"
        )

        show_notification("Logging Started")

    else:

        dpg.set_item_label(
            "logging_button",
            "START LOGGING"
        )

        show_notification("Logging Stopped")


#GUI

dpg.create_context()

large_icon = "D:/Code files pycharm/Masters/magnetic_sensor/adreet_sarkar_app_icon_large.ico"
small_icon = "D:/Code files pycharm/Masters/magnetic_sensor/adreet_sarkar_app_icon.ico"

dpg.create_viewport(
    title="RM3100 Magnetic Field Monitor",
    width=1400,
    height=900,
    large_icon=large_icon,
    small_icon=small_icon
)

max_points = 300

time_data = deque(maxlen=max_points)

bx_data = deque(maxlen=max_points)
by_data = deque(maxlen=max_points)
bz_data = deque(maxlen=max_points)
bmag_data = deque(maxlen=max_points)

notification_start_time = 0


with dpg.window(tag="Primary Window"):

    dpg.add_spacer(height=10)

    ##Header

    with dpg.table(
        header_row=False,
        borders_innerH=False,
        borders_outerH=False,
        borders_innerV=False,
        borders_outerV=False
    ):

        dpg.add_table_column(width_stretch=True)
        dpg.add_table_column(
            width_fixed=True,
            init_width_or_weight=390
        )

        with dpg.table_row():

            dpg.add_text("RM3100 MAGNETIC FIELD MONITOR")

            with dpg.group(horizontal=True):

                dpg.add_text(
                    "●",
                    tag="conn_dot",
                    color=[255, 80, 80, 255]
                )

                dpg.add_text(
                    "DISCONNECTED",
                    tag="conn_label"
                )

                dpg.add_spacer(width=6)

                dpg.add_button(
                    label="⟳",
                    callback=retry_connection,
                    width=28
                )

                dpg.add_button(
                    label="Settings",
                    callback=show_settings_window,
                    width=115
                )

    dpg.add_separator()
    dpg.add_spacer(height=10)

    #Field Values

    with dpg.group(horizontal=True):

        dpg.add_text(
            "Bx : 0.0000 G",
            tag="bx_text"
        )

        dpg.add_spacer(width=50)

        dpg.add_text(
            "By : 0.0000 G",
            tag="by_text"
        )

        dpg.add_spacer(width=50)

        dpg.add_text(
            "Bz : 0.0000 G",
            tag="bz_text"
        )

        dpg.add_spacer(width=50)

        dpg.add_text(
            "|B| : 0.0000 G",
            tag="bmag_text"
        )

    dpg.add_spacer(height=20)

    #Main Area

    with dpg.table(
        header_row=False,
        borders_innerH=False,
        borders_outerH=False,
        borders_innerV=False,
        borders_outerV=False
    ):

        dpg.add_table_column(
            width_fixed=True,
            init_width_or_weight=300
        )

        dpg.add_table_column(width_stretch=True)

        with dpg.table_row():

            #control panel

            with dpg.child_window(
                border=True,
                height=700
            ):

                dpg.add_text("CONTROL PANEL")

                dpg.add_separator()
                dpg.add_spacer(height=15)

                dpg.add_text("Connection Mode")

                dpg.add_text(
                    "WebSocket",
                    tag="mode_text"
                )

                dpg.add_spacer(height=20)

                # Magnetic field PID controller
                dpg.add_text("MAGNETIC FIELD CONTROL")
                dpg.add_separator()
                dpg.add_spacer(height=8)

                dpg.add_text("Bz Setpoint")
                dpg.add_input_float(
                    default_value=0.0,
                    width=200,
                    format="%.4f",
                    tag="setpoint_input",
                    on_enter=True,
                    callback=apply_setpoint
                )

                dpg.add_text(
                    "Target: 0.0000 G",
                    tag="setpoint_status"
                )

                dpg.add_spacer(height=6)

                dpg.add_button(
                    label="APPLY SETPOINT",
                    width=200,
                    callback=apply_setpoint
                )

                dpg.add_spacer(height=8)

                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="START PID",
                        width=95,
                        callback=start_pid
                    )
                    dpg.add_button(
                        label="STOP PID",
                        width=95,
                        callback=stop_pid
                    )

                dpg.add_text(
                    "PID: STOPPED",
                    tag="pid_status",
                    color=[180, 180, 180, 255]
                )

                dpg.add_spacer(height=20)

                dpg.add_button(
                    label="START LOGGING",
                    width=200,
                    callback=toggle_logging,
                    tag="logging_button"
                )

                dpg.add_spacer(height=10)

                dpg.add_button(
                    label="Export Excel",
                    width=200,
                    callback=export_excel,
                    tag="export_button"
                )

                dpg.add_spacer(height=25)

                dpg.add_text("Logging Interval")

                dpg.add_input_int(
                    label="Seconds",
                    default_value=1,
                    width=150,
                    tag="logging_interval"
                )

                dpg.add_text("Display Units")

                dpg.add_combo(
                    items=["G", "mG", "uT", "mT"],
                    default_value="G",
                    width=150,
                    tag="unit_combo",
                    callback=change_unit
                )
                dpg.add_text("Layout")

                dpg.add_combo(
                    items=["All", "Bx", "By", "Bz", "Mag"],
                    default_value="All",
                    width=150,
                    tag="layout_combo",
                    callback=change_layout
                )

                dpg.add_spacer(height=20)

                dpg.add_text("Sensor Calibration")

                dpg.add_button(
                    label="CALIBRATE ZERO",
                    width=200,
                    tag="calibrate_button",
                    callback=start_calibration
                )

                dpg.add_text(
                    "Bias: 0.0000 G",
                    tag="bias_text"
                )


            #Graph

            with dpg.child_window(
                border=True,
                height=640
            ):

                dpg.add_text("LIVE MAGNETIC FIELD")

                dpg.add_separator()
                dpg.add_spacer(height=10)


                with dpg.group(horizontal=True):

                    

                    with dpg.child_window(
                        tag="bx_container",
                        border=True,
                        width=450,
                        height=275
                    ):

                        dpg.add_text("Bx")

                        with dpg.plot(
                            tag="bx_plot",
                            label="Bx",
                            height=245,
                            width=-1
                        ):

                            dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Time (s)",
                                tag="bx_x_axis"
                            )

                            with dpg.plot_axis(
                                dpg.mvYAxis,
                                label="Bx ",
                                tag="bx_y_axis"
                            ):

                                dpg.add_line_series(
                                    [],
                                    [],
                                    label="Bx",
                                    tag="bx_series"
                                )


                    dpg.add_spacer(width=10)


                    

                    with dpg.child_window(
                        tag="by_container",
                        border=True,
                        width=450,
                        height=275
                    ):

                        dpg.add_text("By")

                        with dpg.plot(
                            tag="by_plot",
                            label="By",
                            height=245,
                            width=-1
                        ):

                            dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Time (s)",
                                tag="by_x_axis"
                            )

                            with dpg.plot_axis(
                                dpg.mvYAxis,
                                label="By ",
                                tag="by_y_axis"
                            ):

                                dpg.add_line_series(
                                    [],
                                    [],
                                    label="By",
                                    tag="by_series"
                                )


                dpg.add_spacer(height=10)


                with dpg.group(horizontal=True):

                    

                    with dpg.child_window(
                        tag="bz_container",
                        border=True,
                        width=450,
                        height=275
                    ):

                        dpg.add_text("Bz")

                        with dpg.plot(
                            tag="bz_plot",
                            label="Bz",
                            height=245,
                            width=-1
                        ):

                            dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Time (s)",
                                tag="bz_x_axis"
                            )

                            with dpg.plot_axis(
                                dpg.mvYAxis,
                                label="Bz ",
                                tag="bz_y_axis"
                            ):

                                dpg.add_line_series(
                                    [],
                                    [],
                                    label="Bz",
                                    tag="bz_series"
                                )


                    dpg.add_spacer(width=10)


                    

                    with dpg.child_window(
                        tag="bmag_container",
                        border=True,
                        width=450,
                        height=275
                    ):

                        dpg.add_text("|B|")

                        with dpg.plot(
                            tag="bmag_plot",
                            label="Magnetic Field Magnitude",
                            height=245,
                            width=-1
                        ):

                            dpg.add_plot_axis(
                                dpg.mvXAxis,
                                label="Time (s)",
                                tag="bmag_x_axis"
                            )

                            with dpg.plot_axis(
                                dpg.mvYAxis,
                                label="|B| ",
                                tag="bmag_y_axis"
                            ):

                                dpg.add_line_series(
                                    [],
                                    [],
                                    label="|B|",
                                    tag="bmag_series"
                                )

# Notification Window

with dpg.window(
    label="Notification",
    modal=False,
    show=False,
    no_title_bar=True,
    no_move=True,
    no_resize=True,
    no_close=True,
    tag="notification_popup",
    width=330,
    height=55,
    pos=(1050, 20)
):

    dpg.add_text(
        "",
        tag="notification_text",
        wrap=300
    )

threading.Thread(
    target=websocket_worker,
    daemon=True
).start()

threading.Thread(
    target=usb_monitor,
    daemon=True
).start()


#GUI updating

def update_connectivity_indicator():

    if is_connected:

        dpg.configure_item(
            "conn_dot",
            color=[0, 220, 80, 255]
        )

        if connection_mode == MODE_USB:
            dpg.set_value("conn_label", "USB CONNECTED")
        else:
            dpg.set_value("conn_label", "WEBSOCKET CONNECTED")

    else:

        dpg.configure_item(
            "conn_dot",
            color=[255, 80, 80, 255]
        )

        if connection_mode == MODE_USB:
            if sensor_error:
                dpg.set_value(
                    "conn_label",
                    f"USB: {sensor_error}"
                )
            else:
                dpg.set_value(
                    "conn_label",
                    "USB CONNECTING..."
                )
        else:
            dpg.set_value("conn_label", "WEBSOCKET DISCONNECTED")

    dpg.set_value("mode_text", connection_mode)

    # Show a new error once in a small top-right popup.
    global last_reported_error

    if sensor_error:
        if sensor_error != last_reported_error:
            show_error(sensor_error)
            last_reported_error = sensor_error
    else:
        last_reported_error = None


def update_graph():

    global calibrating, calibration_samples

    current_time = time.time() - start_time
    factor = UNIT_FACTORS[selected_unit]

    # Collect raw readings for zero calibration.
    if calibrating:
        calibration_samples.append((Bx, By, Bz))

        if len(calibration_samples) >= CALIBRATION_COUNT:
            finish_calibration()

    # Apply the stored calibration bias.
    Bx_corrected = Bx - Bx_bias
    By_corrected = By - By_bias
    Bz_corrected = Bz - Bz_bias

    B_magnitude_corrected = (
        Bx_corrected**2 +
        By_corrected**2 +
        Bz_corrected**2
    ) ** 0.5

    time_data.append(current_time)

    bx_data.append(Bx_corrected)
    by_data.append(By_corrected)
    bz_data.append(Bz_corrected)
    bmag_data.append(B_magnitude_corrected)

    dpg.set_value(
        "bx_series",
        [ list(time_data), [v * factor for v in bx_data] ]
    )

    dpg.set_value(
        "by_series",
        [ list(time_data), [v * factor for v in by_data] ]
    )

    dpg.set_value(
        "bz_series",
        [ list(time_data), [v * factor for v in bz_data] ]
    )

    dpg.set_value(
        "bmag_series",
        [ list(time_data), [v * factor for v in bmag_data] ]
    )
    
    if len(time_data) > 2:

        x_min = max(0, current_time - 30)

        dpg.set_axis_limits(
            "bx_x_axis",
            x_min,
            current_time
        )

        dpg.set_axis_limits(
            "by_x_axis",
            x_min,
            current_time
        )

        dpg.set_axis_limits(
            "bz_x_axis",
            x_min,
            current_time
        )

        dpg.set_axis_limits(
            "bmag_x_axis",
            x_min,
            current_time
        )


    dpg.set_axis_limits_auto("bx_y_axis")
    dpg.set_axis_limits_auto("by_y_axis")
    dpg.set_axis_limits_auto("bz_y_axis")
    dpg.set_axis_limits_auto("bmag_y_axis")
    factor = UNIT_FACTORS[selected_unit]

    dpg.set_value(
        "bx_text",
        f"Bx : {Bx_corrected * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "by_text",
        f"By : {By_corrected * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "bz_text",
        f"Bz : {Bz_corrected * factor:.4f} {selected_unit}"
    )

    dpg.set_value(
        "bmag_text",
        f"|B| : {B_magnitude_corrected * factor:.4f} {selected_unit}"
    )


def log_data():

    global last_logged_time

    interval = dpg.get_value(
        "logging_interval"
    )

    current_time = time.time()

    if current_time - last_logged_time >= interval:

        logged_data.append({

            "time_s": round(
                current_time - start_time,
                2
            ),

            "Bx_G": Bx - Bx_bias,
            "By_G": By - By_bias,
            "Bz_G": Bz - Bz_bias,
            "B_magnitude_G": (
                (Bx - Bx_bias)**2 +
                (By - By_bias)**2 +
                (Bz - Bz_bias)**2
            ) ** 0.5,

            "Bx_raw_G": Bx,
            "By_raw_G": By,
            "Bz_raw_G": Bz,
            "B_magnitude_raw_G": B_magnitude,

            "connection": connection_mode

        })

        last_logged_time = current_time


# GUI initilization

try:
    import pyi_splash
    pyi_splash.update_text("Preparing interface...")
except ImportError:
    pyi_splash = None


dpg.setup_dearpygui()

dpg.set_primary_window(
    "Primary Window",
    True
)

try:
    if pyi_splash:
        pyi_splash.update_text("Starting Magnetic Field Monitor...")
except Exception:
    pass

dpg.show_viewport()
dpg.maximize_viewport()
dpg.render_dearpygui_frame()

# Keep splash visible for at least 2 seconds
elapsed = time.time() - start_time
minimum_splash_time = 2.0

if elapsed < minimum_splash_time:
    time.sleep(minimum_splash_time - elapsed)

if pyi_splash:
    pyi_splash.close()


while dpg.is_dearpygui_running():

    # Detect a newly inserted Arduino and show the prompt
    if (
        connection_mode == MODE_WEBSOCKET
        and usb_candidate_port is not None
        and not usb_prompt_shown
        and usb_candidate_port != usb_declined_port
    ):
        show_usb_prompt()

    if time.time() - last_graph_update >= 0.1:

        last_graph_update = time.time()

        update_graph()
        update_connectivity_indicator()

    if is_logging:

        log_data()

        dpg.disable_item(
            "logging_interval"
        )

        dpg.disable_item(
            "export_button"
        )

    else:

        dpg.enable_item(
            "logging_interval"
        )

        dpg.enable_item(
            "export_button"
        )

    if (
        notification_start_time > 0
        and time.time() - notification_start_time > 3
    ):
        dpg.hide_item("notification_popup")

    dpg.render_dearpygui_frame()

with ws_lock:
    active_ws = ws

if active_ws is not None:
    try:
        active_ws.close()
    except Exception:
        pass

with serial_lock:
    active_serial = serial_conn

if active_serial is not None:
    try:
        active_serial.close()
    except Exception:
        pass

dpg.destroy_context()
