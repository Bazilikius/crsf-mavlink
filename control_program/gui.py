import tkinter as tk
from tkinter import ttk, messagebox
import math
from serial_conn import SerialConnection

class ConfiguratorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual YD-RP2040 Mux & Tracker Configurator")
        self.root.geometry("680x700")
        self.root.resizable(False, False)

        self.conn = SerialConnection(on_config_received_cb=self.on_config_received)

        # Configure styles
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', font=('Segoe UI', 10))
        style.configure('TButton', font=('Segoe UI', 10, 'bold'))
        style.configure('Header.TLabel', font=('Segoe UI', 12, 'bold'), foreground='#1A365D')

        self.create_widgets()
        self.refresh_ports()

    def create_widgets(self):
        # Top Frame - Connection Bar
        conn_frame = ttk.LabelFrame(self.root, text=" 1. PC Serial Port Connection ")
        conn_frame.pack(fill="x", padx=15, pady=8)

        ttk.Label(conn_frame, text="COM Port:").pack(side="left", padx=10, pady=10)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.pack(side="left", padx=10, pady=10)

        self.btn_refresh = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.btn_refresh.pack(side="left", padx=5, pady=10)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5, pady=10)

        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", font=('Segoe UI', 10, 'italic'), foreground='red')
        self.lbl_status.pack(side="right", padx=15, pady=10)

        # Notebook for Tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=15, pady=8)

        # Tab 1: Module Switcher Mode Configuration
        tab_mode = ttk.Frame(self.notebook)
        self.notebook.add(tab_mode, text="JR Module Switcher")
        self.setup_switcher_tab(tab_mode)

        # Tab 2: Servo & Antenna Tracker Setup
        tab_tracker = ttk.Frame(self.notebook)
        self.notebook.add(tab_tracker, text="Antenna Tracker Setup")
        self.setup_tracker_tab(tab_tracker)

        # Tab 3: VRX I2C & Remote Channel Settings
        tab_vrx = ttk.Frame(self.notebook)
        self.notebook.add(tab_vrx, text="VRX & Camera Controller")
        self.setup_vrx_tab(tab_vrx)

        # Bottom Console Log Frame
        log_frame = ttk.LabelFrame(self.root, text=" System Console Log ")
        log_frame.pack(fill="both", expand=True, padx=15, pady=10)

        self.txt_log = tk.Text(log_frame, height=6, wrap="word", state="disabled", font=('Courier New', 9))
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=5)

    def log(self, msg):
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", f"[{self.get_time_str()}] {msg}\n")
        self.txt_log.see("end")
        self.txt_log.config(state="disabled")

    def get_time_str(self):
        import datetime
        return datetime.datetime.now().strftime("%H:%M:%S")

    def refresh_ports(self):
        ports = self.conn.list_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
            self.log(f"Found {len(ports)} serial port(s).")
        else:
            self.port_var.set("")
            self.log("No active serial ports found. Connect YD-RP2040 and click Refresh.")

    def toggle_connection(self):
        if self.btn_connect['text'] == "Connect":
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a serial COM port first.")
                return

            if self.conn.connect(port):
                self.btn_connect['text'] = "Disconnect"
                self.lbl_status.config(text="Connected", foreground='green')
                self.log(f"Successfully connected to {port}.")
                self.root.after(200, self.conn.request_config_read)
            else:
                messagebox.showerror("Error", f"Failed to connect to {port}.")
        else:
            self.conn.disconnect()
            self.btn_connect['text'] = "Connect"
            self.lbl_status.config(text="Disconnected", foreground='red')
            self.log("Serial port disconnected.")

    # ------------------- Tab Setup Methods -------------------
    def setup_switcher_tab(self, parent):
        lbl_head = ttk.Label(parent, text="Select JR Module Operational Mode", style="Header.TLabel")
        lbl_head.pack(anchor="w", padx=20, pady=15)

        frame_radio = ttk.Frame(parent)
        frame_radio.pack(fill="x", padx=30, pady=10)

        self.mode_var = tk.IntVar(value=3)

        modes = [
            ("Mode 1: Single JR Module 1 Active (CRSF + MAVLink two-way transmission)", 1),
            ("Mode 2: Single JR Module 2 Active (CRSF only)", 2),
            ("Mode 3: Simultaneous Operation (JR1: MAVLink, JR2: CRSF)", 3)
        ]

        for text, val in modes:
            rb = ttk.Radiobutton(frame_radio, text=text, variable=self.mode_var, value=val, command=self.on_mode_changed)
            rb.pack(anchor="w", pady=10)

        lbl_desc = ttk.Label(parent, text="* Simultaneous operation leverages the split telemetry scheme to send MAVLink through JR1 and CRSF through JR2.", font=('Segoe UI', 9, 'italic'), foreground='gray')
        lbl_desc.pack(anchor="w", padx=20, pady=25)

    def setup_tracker_tab(self, parent):
        # 1. Home coordinates setting
        lbl_home_head = ttk.Label(parent, text="Antenna Tracker Home Coordinates", style="Header.TLabel")
        lbl_home_head.grid(row=0, column=0, columnspan=4, sticky="w", padx=20, pady=10)

        ttk.Label(parent, text="Latitude (°):").grid(row=1, column=0, sticky="e", padx=10, pady=5)
        self.ent_home_lat = ttk.Entry(parent, width=12)
        self.ent_home_lat.grid(row=1, column=1, sticky="w", padx=10, pady=5)
        self.ent_home_lat.insert(0, "0.0")

        ttk.Label(parent, text="Longitude (°):").grid(row=1, column=2, sticky="e", padx=10, pady=5)
        self.ent_home_lon = ttk.Entry(parent, width=12)
        self.ent_home_lon.grid(row=1, column=3, sticky="w", padx=10, pady=5)
        self.ent_home_lon.insert(0, "0.0")

        ttk.Label(parent, text="Altitude (m):").grid(row=2, column=0, sticky="e", padx=10, pady=5)
        self.ent_home_alt = ttk.Entry(parent, width=12)
        self.ent_home_alt.grid(row=2, column=1, sticky="w", padx=10, pady=5)
        self.ent_home_alt.insert(0, "0.0")

        self.btn_set_home = ttk.Button(parent, text="Set Home Position", command=self.on_set_home)
        self.btn_set_home.grid(row=2, column=2, columnspan=2, sticky="ew", padx=10, pady=5)

        # Divider
        div = ttk.Separator(parent, orient="horizontal")
        div.grid(row=3, column=0, columnspan=4, sticky="ew", pady=15)

        # 2. Servo Limits & Calibration
        lbl_servo_head = ttk.Label(parent, text="Servo Calibration & Range Limits (us)", style="Header.TLabel")
        lbl_servo_head.grid(row=4, column=0, columnspan=4, sticky="w", padx=20, pady=10)

        ttk.Label(parent, text="Servo Channel", font=('Segoe UI', 10, 'bold')).grid(row=5, column=0, padx=10, pady=5)
        ttk.Label(parent, text="Minimum (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=1, padx=10, pady=5)
        ttk.Label(parent, text="Maximum (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=2, padx=10, pady=5)
        ttk.Label(parent, text="Trim/Mid (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=3, padx=10, pady=5)

        # Azimuth
        ttk.Label(parent, text="Azimuth (Pan):").grid(row=6, column=0, sticky="e", padx=10, pady=5)
        self.ent_az_min = ttk.Entry(parent, width=10)
        self.ent_az_min.grid(row=6, column=1, padx=10, pady=5)
        self.ent_az_min.insert(0, "1000")

        self.ent_az_max = ttk.Entry(parent, width=10)
        self.ent_az_max.grid(row=6, column=2, padx=10, pady=5)
        self.ent_az_max.insert(0, "2000")

        self.ent_az_trim = ttk.Entry(parent, width=10)
        self.ent_az_trim.grid(row=6, column=3, padx=10, pady=5)
        self.ent_az_trim.insert(0, "1500")

        self.az_rev_var = tk.BooleanVar()
        self.chk_az_rev = ttk.Checkbutton(parent, text="Reverse direction", variable=self.az_rev_var)
        self.chk_az_rev.grid(row=7, column=1, columnspan=2, sticky="w", padx=10, pady=5)

        # Elevation
        ttk.Label(parent, text="Elevation (Tilt):").grid(row=8, column=0, sticky="e", padx=10, pady=5)
        self.ent_el_min = ttk.Entry(parent, width=10)
        self.ent_el_min.grid(row=8, column=1, padx=10, pady=5)
        self.ent_el_min.insert(0, "1000")

        self.ent_el_max = ttk.Entry(parent, width=10)
        self.ent_el_max.grid(row=8, column=2, padx=10, pady=5)
        self.ent_el_max.insert(0, "2000")

        self.ent_el_trim = ttk.Entry(parent, width=10)
        self.ent_el_trim.grid(row=8, column=3, padx=10, pady=5)
        self.ent_el_trim.insert(0, "1500")

        self.el_rev_var = tk.BooleanVar()
        self.chk_el_rev = ttk.Checkbutton(parent, text="Reverse direction", variable=self.el_rev_var)
        self.chk_el_rev.grid(row=9, column=1, columnspan=2, sticky="w", padx=10, pady=5)

        self.btn_save_cal = ttk.Button(parent, text="Upload Calibration", command=self.on_save_calibration)
        self.btn_save_cal.grid(row=10, column=1, columnspan=3, sticky="ew", padx=10, pady=15)

    def setup_vrx_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        # 1. VRX frequency
        lbl_vrx_head = ttk.Label(parent, text="Video Receiver (VRX) Setup", style="Header.TLabel")
        lbl_vrx_head.grid(row=0, column=0, columnspan=2, sticky="w", padx=20, pady=10)

        ttk.Label(parent, text="Remote Control Switch Channel:").grid(row=1, column=0, sticky="e", padx=10, pady=5)
        self.vrx_rc_chan_var = tk.IntVar(value=8)
        self.ent_rc_chan = ttk.Entry(parent, textvariable=self.vrx_rc_chan_var, width=10)
        self.ent_rc_chan.grid(row=1, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(parent, text="VRX Selectable Band:").grid(row=2, column=0, sticky="e", padx=10, pady=5)
        self.band_var = tk.StringVar(value="Fatshark/F")
        self.band_combo = ttk.Combobox(parent, textvariable=self.band_var, values=["Band A", "Band B", "Band E", "Fatshark/F", "Raceband"], width=15, state="readonly")
        self.band_combo.grid(row=2, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(parent, text="VRX Selectable Channel:").grid(row=3, column=0, sticky="e", padx=10, pady=5)
        self.chan_var = tk.IntVar(value=1)
        self.chan_combo = ttk.Combobox(parent, textvariable=self.chan_var, values=[1, 2, 3, 4, 5, 6, 7, 8], width=15, state="readonly")
        self.chan_combo.grid(row=3, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(parent, text="Direct Frequency (MHz):").grid(row=4, column=0, sticky="e", padx=10, pady=5)
        self.ent_freq = ttk.Entry(parent, width=10)
        self.ent_freq.grid(row=4, column=1, sticky="w", padx=10, pady=5)
        self.ent_freq.insert(0, "5740")

        self.btn_update_vrx = ttk.Button(parent, text="Apply VRX Settings", command=self.on_apply_vrx)
        self.btn_update_vrx.grid(row=5, column=0, columnspan=2, sticky="ew", padx=30, pady=10)

        # Divider
        div = ttk.Separator(parent, orient="horizontal")
        div.grid(row=6, column=0, columnspan=2, sticky="ew", pady=10)

        # 2. Camera Switch & Manual Pot Override Settings
        lbl_cam_head = ttk.Label(parent, text="Camera Switch & Manual Potentiometer Override", style="Header.TLabel")
        lbl_cam_head.grid(row=7, column=0, columnspan=2, sticky="w", padx=20, pady=10)

        ttk.Label(parent, text="Cam Switch RC Channel:").grid(row=8, column=0, sticky="e", padx=10, pady=5)
        self.cam_rc_chan_var = tk.IntVar(value=7)
        self.ent_cam_rc_chan = ttk.Entry(parent, textvariable=self.cam_rc_chan_var, width=10)
        self.ent_cam_rc_chan.grid(row=8, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(parent, text="Active Video Feed:").grid(row=9, column=0, sticky="e", padx=10, pady=5)
        self.active_cam_var = tk.StringVar(value="VRX Camera")
        self.cam_combo = ttk.Combobox(parent, textvariable=self.active_cam_var, values=["Analog Camera", "VRX Camera"], width=15, state="readonly")
        self.cam_combo.grid(row=9, column=1, sticky="w", padx=10, pady=5)

        self.pot_override_var = tk.BooleanVar(value=False)
        self.chk_pot_override = ttk.Checkbutton(parent, text="Manual Potentiometer Servo Control Override", variable=self.pot_override_var)
        self.chk_pot_override.grid(row=10, column=0, columnspan=2, sticky="w", padx=40, pady=8)

        self.btn_update_cam = ttk.Button(parent, text="Apply Cam & Pot Settings", command=self.on_apply_cam)
        self.btn_update_cam.grid(row=11, column=0, columnspan=2, sticky="ew", padx=30, pady=10)

        # 3. Live Feedback / Displays
        div2 = ttk.Separator(parent, orient="horizontal")
        div2.grid(row=12, column=0, columnspan=2, sticky="ew", pady=10)

        self.lbl_live_az = ttk.Label(parent, text="Live Azimuth: 0° (360° Limit)", font=('Segoe UI', 11, 'bold'), foreground='#1A365D')
        self.lbl_live_az.grid(row=13, column=0, columnspan=2, pady=5)

        self.lbl_live_el = ttk.Label(parent, text="Live Elevation: 0° (180° Limit)", font=('Segoe UI', 11, 'bold'), foreground='#1A365D')
        self.lbl_live_el.grid(row=14, column=0, columnspan=2, pady=5)

    # ------------------- Action Callbacks -------------------
    def on_mode_changed(self):
        mode = self.mode_var.get()
        if self.conn.set_mode(mode):
            self.log(f"Requested switching mode: {mode}")
        else:
            self.log("Failed to send Mode change command. Check serial connection.")

    def on_set_home(self):
        try:
            lat = float(self.ent_home_lat.get())
            lon = float(self.ent_home_lon.get())
            alt = float(self.ent_home_alt.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric values entered for Home coordinates.")
            return

        if self.conn.set_home_position(lat, lon, alt):
            self.log(f"Set home position: Lat={lat}, Lon={lon}, Alt={alt}")
        else:
            self.log("Failed to send Home position configuration.")

    def on_save_calibration(self):
        try:
            az_min = int(self.ent_az_min.get())
            az_max = int(self.ent_az_max.get())
            az_trim = int(self.ent_az_trim.get())
            az_rev = self.az_rev_var.get()

            el_min = int(self.ent_el_min.get())
            el_max = int(self.ent_el_max.get())
            el_trim = int(self.ent_el_trim.get())
            el_rev = self.el_rev_var.get()
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric values entered for Servo limits.")
            return

        if self.conn.set_calibration(az_min, az_max, az_trim, az_rev, el_min, el_max, el_trim, el_rev):
            self.log("Uploaded Servo Calibration successfully.")
        else:
            self.log("Failed to upload Servo Calibration.")

    def on_apply_vrx(self):
        try:
            rc_chan = int(self.ent_rc_chan.get())
            freq = int(self.ent_freq.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid values entered for VRX configuration.")
            return

        band_str = self.band_var.get()
        band_map = {"Band A": 0, "Band B": 1, "Band E": 2, "Fatshark/F": 3, "Raceband": 4}
        band = band_map.get(band_str, 3)
        chan = int(self.chan_combo.get()) - 1

        if self.conn.set_vrx_config(rc_chan, band, chan, freq):
            self.log(f"Applied VRX config: RC Chan={rc_chan}, Band={band_str}, Chan={chan+1}, Freq={freq}MHz")
        else:
            self.log("Failed to apply VRX configuration.")

    def on_apply_cam(self):
        try:
            cam_rc = int(self.ent_cam_rc_chan.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid value entered for Camera switch RC channel.")
            return

        cam_str = self.active_cam_var.get()
        active_cam = 1 if cam_str == "VRX Camera" else 0
        pot_override = self.pot_override_var.get()

        if self.conn.set_cam_switch_config(active_cam, cam_rc, pot_override):
            self.log(f"Applied Camera and Potentiometer Override Settings: ActiveCam={cam_str}, RCChan={cam_rc}, Override={pot_override}")
        else:
            self.log("Failed to apply Camera switcher and Override configuration.")

    # ------------------- Config Received Event -------------------
    def on_config_received(self, config):
        # Update switcher tab
        self.mode_var.set(config['system_mode'])

        # Update tracker tab calibration
        self.ent_az_min.delete(0, tk.END)
        self.ent_az_min.insert(0, str(config['az_min']))
        self.ent_az_max.delete(0, tk.END)
        self.ent_az_max.insert(0, str(config['az_max']))
        self.ent_az_trim.delete(0, tk.END)
        self.ent_az_trim.insert(0, str(config['az_trim']))
        self.az_rev_var.set(config['az_rev'] == 1)

        self.ent_el_min.delete(0, tk.END)
        self.ent_el_min.insert(0, str(config['el_min']))
        self.ent_el_max.delete(0, tk.END)
        self.ent_el_max.insert(0, str(config['el_max']))
        self.ent_el_trim.delete(0, tk.END)
        self.ent_el_trim.insert(0, str(config['el_trim']))
        self.el_rev_var.set(config['el_rev'] == 1)

        # Update home position
        if config['home_set']:
            self.ent_home_lat.delete(0, tk.END)
            self.ent_home_lat.insert(0, f"{config['home_lat']:.6f}")
            self.ent_home_lon.delete(0, tk.END)
            self.ent_home_lon.insert(0, f"{config['home_lon']:.6f}")
            self.ent_home_alt.delete(0, tk.END)
            self.ent_home_alt.insert(0, f"{config['home_alt']:.1f}")

        # Update VRX settings
        self.ent_rc_chan.delete(0, tk.END)
        self.ent_rc_chan.insert(0, str(config['vrx_rc_chan']))

        band_map_rev = {0: "Band A", 1: "Band B", 2: "Band E", 3: "Fatshark/F", 4: "Raceband"}
        self.band_var.set(band_map_rev.get(config['vrx_band'], "Fatshark/F"))
        self.chan_var.set(config['vrx_chan'] + 1)

        self.ent_freq.delete(0, tk.END)
        self.ent_freq.insert(0, str(config['vrx_mhz']))

        # Update Cam Switch & Pot Overrides
        self.cam_rc_chan_var.set(config['cam_rc_chan'])
        self.active_cam_var.set("VRX Camera" if config['active_camera'] == 1 else "Analog Camera")
        self.pot_override_var.set(config['manual_override'] == 1)

        # Update live feedback display labels
        self.lbl_live_az.config(text=f"Live Azimuth: {config['live_az']}° (360° Limit)")
        self.lbl_live_el.config(text=f"Live Elevation: {config['live_el']}° (180° Limit)")

        self.log(f"Stats Update: AZ={config['live_az']}°, EL={config['live_el']}°, Override={config['manual_override']}, Cam={'VRX' if config['active_camera'] == 1 else 'Analog'}")
