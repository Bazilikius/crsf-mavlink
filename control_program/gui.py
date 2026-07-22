import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import math
import tkintermapview
import serial
from serial_conn import SerialConnection

# FT System 5.8G Frequencies Matrix (11 Bands x 8 Channels = 88 selectable frequencies)
VRX_FREQ_TABLE = [
    [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], # Band A
    [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866], # Band B
    [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], # Band E
    [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880], # Band F (Fatshark)
    [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], # Band Raceband (R)
    [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621], # Band D
    [4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200], # Band X
    [5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613], # Band Lowband (L)
    [4867, 4884, 4921, 4958, 4995, 5032, 5069, 5099], # Band J
    [5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456], # Band U
    [5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600]  # Band O
]

BANDS_LIST = [
    "Band A", "Band B", "Band E", "Fatshark/F", "Raceband",
    "Band D", "Band X", "Lowband/L", "Band J", "Band U", "Band O"
]

class ScrollableFrame(ttk.Frame):
    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(
                scrollregion=self.canvas.bbox("all")
            )
        )

        self.canvas_window = self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # Make sure the scrollable frame expands horizontally to fill the canvas width
        self.canvas.bind('<Configure>', self._on_canvas_configure)

        self.canvas.bind('<Enter>', self._bound_to_mousewheel)
        self.canvas.bind('<Leave>', self._unbound_to_mousewheel)

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self.canvas_window, width=event.width)

    def _bound_to_mousewheel(self, event):
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

    def _unbound_to_mousewheel(self, event):
        self.canvas.unbind_all("<MouseWheel>")
        self.canvas.unbind_all("<Button-4>")
        self.canvas.unbind_all("<Button-5>")

    def _on_mousewheel(self, event):
        if event.num == 4:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(1, "units")
        else:
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

class ConfiguratorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Raspberry Pi Pico Mux & Tracker Configurator")
        self.root.geometry("690x920") # Returned to a compact single-column layout
        self.root.resizable(True, True)

        self.conn = SerialConnection(
            on_config_received_cb=self.on_config_received,
            on_telemetry_received_cb=self.on_telemetry_received,
            log_message_cb=self.log
        )

        # State variables
        self.always_on_top_var = tk.BooleanVar(value=False)
        self.tracking_mode_var = tk.StringVar(value="auto") # "auto" or "manual"
        self.uav_marker = None
        self.home_marker = None
        self.ant_dir_path = None # Rotating real-time antenna direction line path object
        self.last_known_uav_pos = None
        self.last_known_home_pos = None

        # programmatically generated 10px high-contrast solid circular dots
        self.gs_dot_img = None
        self.uav_dot_img = None

        # Interactive Frequency Grid Cells Cache
        self.freq_buttons = {}

        # Configure styles
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', font=('Segoe UI', 10))
        style.configure('TButton', font=('Segoe UI', 10, 'bold'))
        style.configure('Header.TLabel', font=('Segoe UI', 11, 'bold'), foreground='#1A365D')

        self.create_widgets()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        # Single-column compact layout. Main panel widgets go directly onto root frame.
        conn_frame = ttk.LabelFrame(self.root, text=" 1. PC Serial Port Connection ")
        conn_frame.pack(fill="x", padx=15, pady=5)

        ttk.Label(conn_frame, text="COM Port:").pack(side="left", padx=10, pady=8)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.pack(side="left", padx=10, pady=8)

        self.btn_refresh = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.btn_refresh.pack(side="left", padx=5, pady=8)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5, pady=8)

        self.chk_always_on_top = ttk.Checkbutton(
            conn_frame, text="Always on Top", variable=self.always_on_top_var, command=self.toggle_always_on_top
        )
        self.chk_always_on_top.pack(side="left", padx=15, pady=8)

        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", font=('Segoe UI', 10, 'italic'), foreground='red')
        self.lbl_status.pack(side="right", padx=15, pady=8)

        self.lbl_rf_status = ttk.Label(conn_frame, text="RF: OFFLINE", font=('Segoe UI', 10, 'bold'), foreground='red')
        self.lbl_rf_status.pack(side="right", padx=15, pady=8)

        self.lbl_mav_status = ttk.Label(conn_frame, text="MAV: NO DATA", font=('Segoe UI', 10, 'bold'), foreground='red')
        self.lbl_mav_status.pack(side="right", padx=15, pady=8)

        # MAVLink Virtual COM Port Redirector (Bridge)
        redirect_frame = ttk.LabelFrame(self.root, text=" 2. MAVLink Virtual COM Port Redirector (Bridge) ")
        redirect_frame.pack(fill="x", padx=15, pady=5)

        self.redirect_enable_var = tk.BooleanVar(value=False)
        self.chk_redirect = ttk.Checkbutton(redirect_frame, text="Enable MAVLink COM Redirector", variable=self.redirect_enable_var, command=self.on_redirect_toggle)
        self.chk_redirect.pack(side="left", padx=10, pady=8)

        ttk.Label(redirect_frame, text="Redirect to Port:").pack(side="left", padx=10, pady=8)
        self.redirect_port_var = tk.StringVar(value="COM122")
        self.redirect_port_combo = ttk.Combobox(redirect_frame, textvariable=self.redirect_port_var, width=12)
        self.redirect_port_combo.pack(side="left", padx=5, pady=8)

        self.lbl_redirect_status = ttk.Label(redirect_frame, text="Redirector: Idle", font=('Segoe UI', 10, 'italic'), foreground='gray')
        self.lbl_redirect_status.pack(side="right", padx=15, pady=8)

        # Notebook for Tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=15, pady=5)

        # Tab 1: Module Switcher Mode Configuration (Scrollable)
        self.tab_mode_scroll = ScrollableFrame(self.notebook)
        self.notebook.add(self.tab_mode_scroll, text="JR Module Switcher")
        self.setup_switcher_tab(self.tab_mode_scroll.scrollable_frame)

        # Tab 2: Servo & Antenna Tracker Setup (Scrollable)
        self.tab_tracker_scroll = ScrollableFrame(self.notebook)
        self.notebook.add(self.tab_tracker_scroll, text="Antenna Tracker Setup")
        self.setup_tracker_tab(self.tab_tracker_scroll.scrollable_frame)

        # Tab 3: VRX I2C & Remote Channel Settings (Scrollable)
        self.tab_vrx_scroll = ScrollableFrame(self.notebook)
        self.notebook.add(self.tab_vrx_scroll, text="VRX & Switches Config")
        self.setup_vrx_tab(self.tab_vrx_scroll.scrollable_frame)

        # Bottom Console Log Frame
        log_frame = ttk.LabelFrame(self.root, text=" System Console Log ")
        log_frame.pack(fill="x", expand=False, padx=15, pady=5)

        self.txt_log = tk.Text(log_frame, height=5, wrap="word", state="disabled", font=('Courier New', 9))
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=5)

    def make_dot_image(self, color):
        # Create a programmatically generated high-contrast 10-pixel solid circular icon
        img = tk.PhotoImage(width=10, height=10)
        # circle equation: (x-4.5)^2 + (y-4.5)^2 <= 5.0^2
        for y in range(10):
            for x in range(10):
                if (x - 4.5)**2 + (y - 4.5)**2 <= 25.0:
                    img.put(color, (x, y))
        return img

    def on_freq_cell_clicked(self, band_idx, chan_idx):
        # Handle custom channel selection mapping logic depending on current mode and position toggle settings
        current_mode_str = self.vrx_mode_var.get()
        if current_mode_str == "S2 + 6POS (Simultaneous)":
            self.log("Cannot select frequency manually in S2 + 6POS mode. Use your transmitter S2/6POS toggle switches!")
            messagebox.showwarning(
                "Manual Selection Disabled",
                "Video channel manual selection is disabled in 'S2 + 6POS (Simultaneous)' control mode.\n\n"
                "In this mode, frequencies are adjusted dynamically by your physical transmitter dials/switches.",
                parent=self.root
            )
            return

        if current_mode_str == "Custom Positions Table Mapping" or current_mode_str == "6POS Only (Video Band)":
            # If 6POS or table mapping mode is selected, we click on the necessary channels in the table to select them,
            # then click upload/ok. Let's build a selection queue matching the switch positions count.
            # Number of channels to select depends on how many positions are defined or active.
            # For 6POS mode, that is exactly 6 positions. For custom positions table, it depends on vrx_positions_count.
            if current_mode_str == "6POS Only (Video Band)":
                num_positions = 6
            else:
                try:
                    num_positions = self.vrx_pos_count_var.get()
                except Exception:
                    num_positions = 3
            num_positions = max(2, min(num_positions, 8))

            # Use self.selected_mapping_queue to manage a list of selected (band_idx, chan_idx) tuples
            if not hasattr(self, 'selected_mapping_queue'):
                self.selected_mapping_queue = []

            cell = (band_idx, chan_idx)
            if cell in self.selected_mapping_queue:
                # Toggle deselect if already clicked!
                self.selected_mapping_queue.remove(cell)
                self.log(f"Deselected frequency cell {BANDS_LIST[band_idx]} Ch {chan_idx+1}")
            else:
                # Add to queue
                if len(self.selected_mapping_queue) >= num_positions:
                    # Remove the oldest to keep size under num_positions
                    oldest = self.selected_mapping_queue.pop(0)
                    self.log(f"Deselected oldest cell {BANDS_LIST[oldest[0]]} Ch {oldest[1]+1} to make room")
                self.selected_mapping_queue.append(cell)
                self.log(f"Selected frequency cell {BANDS_LIST[band_idx]} Ch {chan_idx+1} (Pos {len(self.selected_mapping_queue)})")

            # Update mapping table fields
            for i in range(num_positions):
                if i < len(self.selected_mapping_queue):
                    b, c = self.selected_mapping_queue[i]
                    self.row_widgets[i]['band_var'].set(BANDS_LIST[b])
                    self.row_widgets[i]['chan_var'].set(c + 1)
                    self.on_table_row_changed(i)

            # Refresh grid cells highlighting
            self.highlight_active_freq_cell(self.conn.last_config.get('vrx_band', 0) if hasattr(self.conn, 'last_config') else 0,
                                            self.conn.last_config.get('vrx_chan', 0) if hasattr(self.conn, 'last_config') else 0)
            return

        freq_mhz = VRX_FREQ_TABLE[band_idx][chan_idx]
        band_name = BANDS_LIST[band_idx]
        self.log(f"Selecting Video Frequency cell: {band_name}, Channel {chan_idx+1} ({freq_mhz} MHz)")

        # 2. Transmit Command 0x45 over serial to tune the FT System 5.8G receiver immediately
        payload = [0x45, band_idx, chan_idx]
        if self.conn.send_command(payload):
            # Highlight this clicked cell immediately for zero-latency UI updates!
            self.highlight_active_freq_cell(band_idx, chan_idx)
        else:
            self.log("Failed to send direct frequency set command. Check connection.")

    def highlight_active_freq_cell(self, active_band, active_chan):
        # Scan and update backgrounds of all cells
        current_mode_str = self.vrx_mode_var.get()
        has_queue = hasattr(self, 'selected_mapping_queue')

        for (b, c), btn in self.freq_buttons.items():
            if (current_mode_str == "Custom Positions Table Mapping" or current_mode_str == "6POS Only (Video Band)") and has_queue:
                # Highlight active selected mapping queue channels in order
                if (b, c) in self.selected_mapping_queue:
                    idx = self.selected_mapping_queue.index((b, c))
                    btn.config(bg="#BEE3F8", text=f"P{idx+1}", relief="sunken", font=('Segoe UI', 8, 'bold'))
                else:
                    btn.config(bg="#F0F0F0", text=str(VRX_FREQ_TABLE[b][c]), relief="groove", font=('Segoe UI', 8))
            else:
                # Standard mode cell highlighting
                if b == active_band and c == active_chan:
                    # Active cell gets high-contrast green highlight background!
                    btn.config(bg="#90EE90", text=str(VRX_FREQ_TABLE[b][c]), relief="sunken", font=('Segoe UI', 8, 'bold'))
                else:
                    btn.config(bg="#F0F0F0", text=str(VRX_FREQ_TABLE[b][c]), relief="groove", font=('Segoe UI', 8))

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
        self.redirect_port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
            self.log(f"Found {len(ports)} serial port(s).")
        else:
            self.port_var.set("")
            self.log("No active serial ports found. Connect Pico and click Refresh.")

    def toggle_connection(self):
        if self.btn_connect['text'] == "Connect":
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a serial COM port first.")
                return

            # Sync user's custom MAVP2P ports before connecting so they take effect immediately!
            try:
                self.conn.udp_port_sec = int(self.mav_in_port_var.get())
                self.conn.udp_tx_port_sec = int(self.mav_out_port_var.get())
            except Exception:
                pass

            redirect_port = None
            if self.redirect_enable_var.get():
                redirect_port = self.redirect_port_var.get().strip()

            # Before connecting, verify the redirect port if selected
            if redirect_port:
                # We can check if it exists in list_ports or try to open/test it, or just let connect do it.
                # Let's see if we can catch errors from within connect or handle them.
                pass

            if self.conn.connect(port, redirect_port=redirect_port):
                self.btn_connect['text'] = "Disconnect"
                self.lbl_status.config(text="Connected", foreground='green')
                self.log(f"Successfully connected to {port}.")

                if redirect_port:
                    if self.conn.mav_redirect_ser and self.conn.mav_redirect_ser.is_open:
                        self.lbl_redirect_status.config(text="Redirector: Active", foreground='green')
                    else:
                        self.lbl_redirect_status.config(text="Redirector: Failed", foreground='red')
                        self.show_redirector_help(redirect_port)
                else:
                    self.lbl_redirect_status.config(text="Redirector: Idle", foreground='gray')

                self.root.after(200, self.conn.request_config_read)
                # Start periodic PC presence heartbeat to keep Board 1 in PC_MULTIPLEXED mode
                self.root.after(2000, self._periodic_heartbeat)
            else:
                messagebox.showerror("Error", f"Failed to connect to {port}.")
        else:
            self.conn.disconnect()
            self.btn_connect['text'] = "Connect"
            self.lbl_status.config(text="Disconnected", foreground='red')
            self.lbl_rf_status.config(text="RF: OFFLINE", foreground='red')
            self.lbl_mav_status.config(text="MAV: NO DATA", foreground='red')
            self.lbl_redirect_status.config(text="Redirector: Idle", foreground='gray')
            self.log("Serial port disconnected.")

    def show_redirector_help(self, redirect_port):
        messagebox.showwarning(
            "Redirector Setup Required",
            f"Could not open virtual redirect port '{redirect_port}' (FileNotFoundError / COM port does not exist).\n\n"
            "To redirect MAVLink telemetry to a virtual COM port in Windows:\n"
            "1. You must first install virtual COM port emulation software like 'com0com' or VSPE.\n"
            "2. Create a virtual COM port pair (e.g., COM122 <-> COM123).\n"
            "3. Select COM122 as the redirect port in this GUI, and connect your Ground Control Station (GCS) to COM123.\n\n"
            "Alternatively, use the built-in UDP proxy: simply configure Mission Planner / QGroundControl to connect via "
            "UDP to port 14550 on 127.0.0.1, which works automatically without virtual COM ports!",
            parent=self.root
        )

    def on_redirect_toggle(self):
        # If already connected, we can dynamically start or stop redirector!
        if self.btn_connect['text'] == "Disconnect":
            if self.redirect_enable_var.get():
                redirect_port = self.redirect_port_var.get().strip()
                if redirect_port:
                    # If already active, close first
                    if self.conn.mav_redirect_ser and self.conn.mav_redirect_ser.is_open:
                        try:
                            self.conn.mav_redirect_ser.close()
                        except Exception:
                            pass
                    try:
                        self.conn.mav_redirect_port = redirect_port
                        self.conn.mav_redirect_ser = serial.Serial(redirect_port, baudrate=115200, timeout=0.1)
                        import threading
                        self.conn.mav_redirect_thread = threading.Thread(target=self.conn._redirect_loop, daemon=True)
                        self.conn.mav_redirect_thread.start()
                        self.lbl_redirect_status.config(text="Redirector: Active", foreground='green')
                        self.log(f"MAVLink COM Redirector successfully started on {redirect_port}.")
                    except Exception as e:
                        self.conn.mav_redirect_ser = None
                        self.lbl_redirect_status.config(text="Redirector: Failed", foreground='red')
                        self.log(f"Warning: Could not open MAVLink Redirector port {redirect_port} ({e}).")
                        self.show_redirector_help(redirect_port)
            else:
                # Stop redirector
                if self.conn.mav_redirect_ser:
                    try:
                        self.conn.mav_redirect_ser.close()
                    except Exception:
                        pass
                self.conn.mav_redirect_ser = None
                self.lbl_redirect_status.config(text="Redirector: Idle", foreground='gray')
                self.log("MAVLink COM Redirector stopped.")

    def toggle_always_on_top(self):
        state = self.always_on_top_var.get()
        self.root.attributes("-topmost", state)
        self.log(f"Always on Top set to: {state}")

    def on_tracking_mode_changed(self):
        mode = self.tracking_mode_var.get()
        # Toggle tracking mode between Auto and Manual Control!
        manual_val = 1 if mode == "manual" else 0
        if self.conn.set_cam_switch_config(
            active_camera=1 if self.active_cam_var.get() == "VRX Camera" else 0,
            cam_rc_channel=self.cam_rc_chan_var.get(),
            manual_override=manual_val
        ):
            self.pot_override_var.set(mode == "manual")
            self.log(f"Tracking mode switched to: {mode.upper()} tracking.")
        else:
            self.log("Failed to send tracking mode change command.")

    def on_calibrate_azimuth_zero(self):
        # 1. Ask the user for the target calibration reference azimuth angle (Step 1 of Wizard)
        val = simpledialog.askinteger(
            "Azimuth Calibration",
            "Step 1: Enter desired target calibration reference angle in degrees (0 - 359):\n"
            "(e.g., enter '0' for North alignment, '90' for East, etc.)",
            parent=self.root,
            minvalue=0,
            maxvalue=359,
            initialvalue=0
        )
        if val is None:
            self.log("Azimuth calibration wizard cancelled by user.")
            return

        # 2. Inform user they must point/drive the tracker to match this alignment (Step 2 of Wizard)
        messagebox.showinfo(
            "Drive Antenna Tracker",
            f"Step 2: Please point/drive the physical antenna tracker until it is physically aligned with your target reference of {val}°.\n\n"
            "Once aligned, click OK to proceed to final calibration confirmation.",
            parent=self.root
        )

        # 3. Final alignment confirmation before applying (Step 3 of Wizard)
        confirm = messagebox.askyesno(
            "Confirm Calibration Offset Alignment",
            f"Step 3: Confirm tracker is correctly pointing to {val}°?\n\n"
            "Clicking YES will lock this current physical potentiometer position as representing the reference angle.",
            parent=self.root
        )
        if not confirm:
            self.log("Azimuth calibration offset cancelled at confirmation stage.")
            return

        # Transmit command 0x80 to lock calibration
        if self.conn.calibrate_azimuth_zero(ref_deg=val):
            self.log(f"Successfully locked calibration offset. Current position is calibrated to {val}°.")
            messagebox.showinfo(
                "Calibration Success",
                f"Antenna tracker successfully calibrated!\n\n"
                f"Current direction has been mapped to exactly {val}°.",
                parent=self.root
            )
        else:
            self.log("Failed to send Calibrate Azimuth command.")
            messagebox.showerror(
                "Error",
                "Failed to send calibration command. Please check serial connection status.",
                parent=self.root
            )

    # ------------------- Telemetry & Mapping Updates -------------------
    def on_telemetry_received(self, data):
        self.root.after(0, self._on_telemetry_received_main_thread, data)

    def _on_telemetry_received_main_thread(self, data):
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        alt = data.get('alt', 0.0)

        # Log & Update Drone Position Marker
        if self.uav_marker:
            self.uav_marker.set_position(lat, lon)
        else:
            # Use 10px circular icon programmatically generated to satisfy "reduce the antenna markers to 10 pixels"
            # Cascading fallback for tkintermapview version compatibility (handles both icon and image arguments)
            try:
                self.uav_marker = self.map_view.set_marker(lat, lon, text="Drone Position", icon=self.uav_dot_img)
            except TypeError:
                try:
                    self.uav_marker = self.map_view.set_marker(lat, lon, text="Drone Position", image=self.uav_dot_img)
                except Exception:
                    self.uav_marker = self.map_view.set_marker(lat, lon, text="Drone Position")

            if self.map_follow_drone_var.get():
                self.map_view.set_position(lat, lon)

        if self.map_follow_drone_var.get():
            self.map_view.set_position(lat, lon)

        self.last_known_uav_pos = (lat, lon)

        # === Automatic Antenna Tracking calculations ===
        if self.tracking_mode_var.get() == "auto" and self.last_known_home_pos:
            home_lat, home_lon = self.last_known_home_pos
            try:
                home_alt = float(self.ent_home_alt.get())
            except Exception:
                home_alt = 0.0

            # Convert coordinates to radians
            lat1 = math.radians(home_lat)
            lon1 = math.radians(home_lon)
            lat2 = math.radians(lat)
            lon2 = math.radians(lon)

            # 1. Azimuth (Bearing) Calculation
            dlon = lon2 - lon1
            y = math.sin(dlon) * math.cos(lat2)
            x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
            bearing = math.degrees(math.atan2(y, x))
            azimuth = (bearing + 360) % 360

            # 2. Elevation (Pitch) Calculation
            R = 6371000.0 # Earth Radius in meters
            dlat = lat2 - lat1
            a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            ground_dist = R * c

            alt_diff = alt - home_alt
            elevation = math.degrees(math.atan2(alt_diff, ground_dist)) if ground_dist > 0.1 else 0.0
            elevation = max(-10.0, min(elevation, 90.0))

            # 3. Map tracking angles to Servo Microseconds (based on configured calibration parameters)
            try:
                az_min = int(self.ent_az_min.get())
                az_max = int(self.ent_az_max.get())
                az_trim = int(self.ent_az_trim.get())
                az_reversed = self.az_rev_var.get()

                el_min = int(self.ent_el_min.get())
                el_max = int(self.ent_el_max.get())
                el_trim = int(self.ent_el_trim.get())
                el_reversed = self.el_rev_var.get()
            except Exception:
                az_min, az_max, az_trim, az_reversed = 1000, 2000, 1500, False
                el_min, el_max, el_trim, el_reversed = 1000, 2000, 1500, False

            # Azimuth Servo Mapping
            az_pct = azimuth / 360.0
            if az_reversed:
                az_pct = 1.0 - az_pct
            az_us = az_min + int(az_pct * (az_max - az_min))

            # Elevation Servo Mapping
            # Standard elevation range spans -10 degrees to 90 degrees (100 degrees total span)
            el_pct = (elevation + 10.0) / 100.0
            if el_reversed:
                el_pct = 1.0 - el_pct
            el_us = el_min + int(el_pct * (el_max - el_min))

            # Clamp limits safely
            az_us = max(az_min, min(az_us, az_max))
            el_us = max(el_min, min(el_us, el_max))

            # Send computed direct servo microseconds Command 0x70 back to Board 1 to forward to Board 2
            payload = [
                0x70,
                (az_us >> 8) & 0xFF, az_us & 0xFF,
                (el_us >> 8) & 0xFF, el_us & 0xFF
            ]
            if self.conn.send_command(payload):
                self.log(f"Auto-tracking Servos Updated: AZ={azimuth:.1f}° ({az_us}us), EL={elevation:.1f}° ({el_us}us)")

    def _update_antenna_direction_line(self, home_lat, home_lon, azimuth_deg):
        # Calculate real-time heading endpoint path on map using spherical trigonometry
        # To make it only for a distance of exactly 30km on the map scale:
        # 30 km = 30000 meters / 111139 meters per degree = 0.269932 degrees of latitude.
        distance_deg = 0.269932
        rad = math.radians(azimuth_deg)

        # Calculate target endpoint coordinates
        target_lat = home_lat + (distance_deg * math.cos(rad))
        # Account for latitude shrinking longitude spacing
        lat_scale = math.cos(math.radians(home_lat))
        if abs(lat_scale) < 0.01:
            lat_scale = 1.0
        target_lon = home_lon + ((distance_deg * math.sin(rad)) / lat_scale)

        # Clear previous path to avoid cluttering the display
        if self.ant_dir_path:
            try:
                self.ant_dir_path.delete()
            except Exception:
                pass

        # Draw high-visibility line from GS center coordinates in real time pointing along raw azimuth
        try:
            self.ant_dir_path = self.map_view.set_path(
                [(home_lat, home_lon), (target_lat, target_lon)],
                color="#00FF00", # Neon Green high contrast path line
                width=3
            )
        except Exception as e:
            self.log(f"Map draw path error: {e}")

    # ------------------- Tab Setup Methods -------------------
    def setup_switcher_tab(self, parent):
        lbl_head = ttk.Label(parent, text="Select JR Module Operational Mode", style="Header.TLabel")
        lbl_head.pack(anchor="w", padx=20, pady=15)

        frame_radio = ttk.Frame(parent)
        frame_radio.pack(fill="x", padx=30, pady=10)

        self.mode_var = tk.IntVar(value=3)

        modes = [
            ("Mode 1: Single JR Module 1 Active (CRSF + MAVLink separate interfaces)", 1),
            ("Mode 2: Single JR Module 2 Active (CRSF only)", 2),
            ("Mode 3: Simultaneous Operation (JR1: MAVLink, JR2: CRSF)", 3)
        ]

        for text, val in modes:
            rb = ttk.Radiobutton(frame_radio, text=text, variable=self.mode_var, value=val, command=self.on_mode_changed)
            rb.pack(anchor="w", pady=10)

        lbl_desc = ttk.Label(parent, text="* Simultaneous operation leverages the split telemetry scheme to send MAVLink through JR1 and CRSF through JR2.", font=('Segoe UI', 9, 'italic'), foreground='gray')
        lbl_desc.pack(anchor="w", padx=20, pady=10)

        # --- Baudrates configuration section ---
        baud_frame = ttk.LabelFrame(parent, text=" Configure JR Modules Baudrates ")
        baud_frame.pack(fill="x", padx=30, pady=10)

        baud_opts = ["9600", "19200", "38400", "57600", "115200", "230400", "400000", "420000", "460800", "921600"]

        ttk.Label(baud_frame, text="JR1 CRSF Baudrate:").grid(row=0, column=0, sticky="e", padx=10, pady=10)
        self.jr1_crsf_baud_var = tk.StringVar(value="400000")
        self.combo_jr1_crsf = ttk.Combobox(baud_frame, textvariable=self.jr1_crsf_baud_var, values=baud_opts, width=12, state="readonly")
        self.combo_jr1_crsf.grid(row=0, column=1, sticky="w", padx=10, pady=10)

        ttk.Label(baud_frame, text="JR1 MAVLink Baudrate:").grid(row=1, column=0, sticky="e", padx=10, pady=10)
        self.jr1_mav_baud_var = tk.StringVar(value="115200")
        self.combo_jr1_mav = ttk.Combobox(baud_frame, textvariable=self.jr1_mav_baud_var, values=baud_opts, width=12, state="readonly")
        self.combo_jr1_mav.grid(row=1, column=1, sticky="w", padx=10, pady=10)

        ttk.Label(baud_frame, text="JR2 CRSF Baudrate:").grid(row=2, column=0, sticky="e", padx=10, pady=10)
        self.jr2_crsf_baud_var = tk.StringVar(value="400000")
        self.combo_jr2_crsf = ttk.Combobox(baud_frame, textvariable=self.jr2_crsf_baud_var, values=baud_opts, width=12, state="readonly")
        self.combo_jr2_crsf.grid(row=2, column=1, sticky="w", padx=10, pady=10)

        # Apply Button
        self.btn_apply_switcher = ttk.Button(parent, text="Apply Switcher & Baudrate Settings", command=self.on_apply_switcher_settings)
        self.btn_apply_switcher.pack(padx=30, pady=15)

    def setup_tracker_tab(self, parent):
        lbl_home_head = ttk.Label(parent, text="Antenna Tracker Home Coordinates", style="Header.TLabel")
        lbl_home_head.grid(row=0, column=0, columnspan=4, sticky="w", padx=20, pady=5)

        ttk.Label(parent, text="Latitude (°):").grid(row=1, column=0, sticky="e", padx=10, pady=2)
        self.ent_home_lat = ttk.Entry(parent, width=12)
        self.ent_home_lat.grid(row=1, column=1, sticky="w", padx=10, pady=2)
        self.ent_home_lat.insert(0, "0.0")

        ttk.Label(parent, text="Longitude (°):").grid(row=1, column=2, sticky="e", padx=10, pady=2)
        self.ent_home_lon = ttk.Entry(parent, width=12)
        self.ent_home_lon.grid(row=1, column=3, sticky="w", padx=10, pady=2)
        self.ent_home_lon.insert(0, "0.0")

        ttk.Label(parent, text="Altitude (m):").grid(row=2, column=0, sticky="e", padx=10, pady=2)
        self.ent_home_alt = ttk.Entry(parent, width=12)
        self.ent_home_alt.grid(row=2, column=1, sticky="w", padx=10, pady=2)
        self.ent_home_alt.insert(0, "0.0")

        self.btn_set_home = ttk.Button(parent, text="Set Home Position", command=self.on_set_home)
        self.btn_set_home.grid(row=2, column=2, columnspan=2, sticky="ew", padx=10, pady=2)

        # Divider
        div = ttk.Separator(parent, orient="horizontal")
        div.grid(row=3, column=0, columnspan=4, sticky="ew", pady=10)

        # Servo Limits & Calibration
        lbl_servo_head = ttk.Label(parent, text="Servo Calibration & Range Limits (us)", style="Header.TLabel")
        lbl_servo_head.grid(row=4, column=0, columnspan=4, sticky="w", padx=20, pady=5)

        ttk.Label(parent, text="Servo Channel", font=('Segoe UI', 10, 'bold')).grid(row=5, column=0, padx=10, pady=2)
        ttk.Label(parent, text="Minimum (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=1, padx=10, pady=2)
        ttk.Label(parent, text="Maximum (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=2, padx=10, pady=2)
        ttk.Label(parent, text="Trim/Mid (us)", font=('Segoe UI', 10, 'bold')).grid(row=5, column=3, padx=10, pady=2)

        # Azimuth
        ttk.Label(parent, text="Azimuth (Pan):").grid(row=6, column=0, sticky="e", padx=10, pady=2)
        self.ent_az_min = ttk.Entry(parent, width=10)
        self.ent_az_min.grid(row=6, column=1, padx=10, pady=2)
        self.ent_az_min.insert(0, "1000")

        self.ent_az_max = ttk.Entry(parent, width=10)
        self.ent_az_max.grid(row=6, column=2, padx=10, pady=2)
        self.ent_az_max.insert(0, "2000")

        self.ent_az_trim = ttk.Entry(parent, width=10)
        self.ent_az_trim.grid(row=6, column=3, padx=10, pady=2)
        self.ent_az_trim.insert(0, "1500")

        self.az_rev_var = tk.BooleanVar()
        self.chk_az_rev = ttk.Checkbutton(parent, text="Reverse direction", variable=self.az_rev_var)
        self.chk_az_rev.grid(row=7, column=1, columnspan=2, sticky="w", padx=10, pady=2)

        # Elevation
        ttk.Label(parent, text="Elevation (Tilt):").grid(row=8, column=0, sticky="e", padx=10, pady=2)
        self.ent_el_min = ttk.Entry(parent, width=10)
        self.ent_el_min.grid(row=8, column=1, padx=10, pady=2)
        self.ent_el_min.insert(0, "1000")

        self.ent_el_max = ttk.Entry(parent, width=10)
        self.ent_el_max.grid(row=8, column=2, padx=10, pady=2)
        self.ent_el_max.insert(0, "2000")

        self.ent_el_trim = ttk.Entry(parent, width=10)
        self.ent_el_trim.grid(row=8, column=3, padx=10, pady=2)
        self.ent_el_trim.insert(0, "1500")

        self.el_rev_var = tk.BooleanVar()
        self.chk_el_rev = ttk.Checkbutton(parent, text="Reverse direction", variable=self.el_rev_var)
        self.chk_el_rev.grid(row=9, column=1, columnspan=2, sticky="w", padx=10, pady=2)

        self.btn_save_cal = ttk.Button(parent, text="Upload Calibration", command=self.on_save_calibration)
        self.btn_save_cal.grid(row=10, column=1, columnspan=3, sticky="ew", padx=10, pady=5)

        # 2. Embedded Dynamic Map and Track Control Settings (50% scale relative to original layout)
        div_map = ttk.Separator(parent, orient="horizontal")
        div_map.grid(row=11, column=0, columnspan=4, sticky="ew", pady=5)

        # Map tracking control bar inside Tab 2
        map_hdr_frame = ttk.Frame(parent)
        map_hdr_frame.grid(row=12, column=0, columnspan=4, sticky="ew", padx=15, pady=2)

        ttk.Label(map_hdr_frame, text="Tracking:", font=('Segoe UI', 9, 'bold')).pack(side="left", padx=2)

        self.rb_auto = ttk.Radiobutton(
            map_hdr_frame, text="Auto", variable=self.tracking_mode_var, value="auto", command=self.on_tracking_mode_changed
        )
        self.rb_auto.pack(side="left", padx=5)

        self.rb_manual = ttk.Radiobutton(
            map_hdr_frame, text="Manual", variable=self.tracking_mode_var, value="manual", command=self.on_tracking_mode_changed
        )
        self.rb_manual.pack(side="left", padx=5)

        self.map_follow_drone_var = tk.BooleanVar(value=True)
        self.chk_follow_drone = ttk.Checkbutton(
            map_hdr_frame, text="Map Follows Drone", variable=self.map_follow_drone_var
        )
        self.chk_follow_drone.pack(side="left", padx=10)

        self.btn_cal_set = ttk.Button(
            map_hdr_frame, text="Calibrate AZ (Set Cur)", command=self.on_calibrate_azimuth_zero, style="TButton"
        )
        self.btn_cal_set.pack(side="right", padx=2)

        # Live tracking feedback displays inside Tab 2
        feedback_frame = ttk.Frame(parent)
        feedback_frame.grid(row=13, column=0, columnspan=4, sticky="ew", padx=15, pady=2)

        self.lbl_live_az = ttk.Label(feedback_frame, text="Live Azimuth: 0° (360° Limit)", font=('Segoe UI', 10, 'bold'), foreground='#1A365D')
        self.lbl_live_az.pack(side="left", expand=True, padx=10)

        self.lbl_live_el = ttk.Label(feedback_frame, text="Live Elevation: 0° (180° Limit)", font=('Segoe UI', 10, 'bold'), foreground='#1A365D')
        self.lbl_live_el.pack(side="right", expand=True, padx=10)

        # Map widget (reduced size by approximately 50%)
        # Original size was full panel, now reduced to width=450, height=245.
        self.map_view = tkintermapview.TkinterMapView(parent, width=450, height=245, corner_radius=10)
        self.map_view.grid(row=14, column=0, columnspan=4, sticky="nsew", padx=20, pady=5)

        # Generate 10px circular markers programmatically to fulfill "reduce the antenna markers to 10 pixels"
        self.gs_dot_img = self.make_dot_image("blue")
        self.uav_dot_img = self.make_dot_image("red")

        # Set Google Satellite Hybrid Map to display satellite imagery overlaid with roads, city labels, and country borders.
        # lyrs=y is the standard Google Maps layer code for hybrid satellite views with roads/labels/borders.
        self.map_view.set_tile_server("https://mt0.google.com/vt/lyrs=y&x={x}&y={y}&z={z}", max_zoom=22)
        self.map_view.set_zoom(15)

        # Start at default position
        self.map_view.set_position(0.0, 0.0)

    def setup_vrx_tab(self, parent):
        # 1. Video Control Mode Frame
        mode_frame = ttk.LabelFrame(parent, text=" VRX Control Method Selection ")
        mode_frame.grid(row=0, column=0, columnspan=4, sticky="ew", padx=15, pady=5)

        ttk.Label(mode_frame, text="Active Control Mode:").grid(row=0, column=0, sticky="e", padx=10, pady=5)
        self.vrx_mode_var = tk.StringVar(value="S2 + 6POS (Simultaneous)")
        self.combo_vrx_mode = ttk.Combobox(mode_frame, textvariable=self.vrx_mode_var, values=["S2 Only (Video Channel)", "6POS Only (Video Band)", "S2 + 6POS (Simultaneous)", "Custom Positions Table Mapping"], width=30, state="readonly")
        self.combo_vrx_mode.grid(row=0, column=1, columnspan=2, sticky="w", padx=10, pady=5)
        self.combo_vrx_mode.bind("<<ComboboxSelected>>", lambda e: self.on_vrx_control_mode_switched())

        # 1a. Interactive Video Frequency Table Grid (Click to Select)
        grid_frame = ttk.LabelFrame(parent, text=" Interactive Video Frequency Grid Table (Click to Select) ")
        grid_frame.grid(row=1, column=0, columnspan=4, sticky="ew", padx=15, pady=5)

        # Grid Headers for Channels 1 to 8
        for col_idx in range(8):
            lbl_ch = ttk.Label(grid_frame, text=f"Ch {col_idx+1}", font=('Segoe UI', 9, 'bold'))
            lbl_ch.grid(row=0, column=col_idx+1, padx=4, pady=2)

        # Draw 11 Bands x 8 Channels
        for b_idx in range(11):
            lbl_band = ttk.Label(grid_frame, text=BANDS_LIST[b_idx], font=('Segoe UI', 9, 'bold'), width=12, anchor="w")
            lbl_band.grid(row=b_idx+1, column=0, padx=6, pady=2, sticky="w")

            for c_idx in range(8):
                freq_val = VRX_FREQ_TABLE[b_idx][c_idx]
                btn_freq = tk.Button(
                    grid_frame,
                    text=str(freq_val),
                    font=('Segoe UI', 8),
                    width=5,
                    relief="groove",
                    borderwidth=1,
                    bg="#F0F0F0",
                    activebackground="#90EE90",
                    command=lambda b=b_idx, c=c_idx: self.on_freq_cell_clicked(b, c)
                )
                btn_freq.grid(row=b_idx+1, column=c_idx+1, padx=3, pady=2)
                self.freq_buttons[(b_idx, c_idx)] = btn_freq

        # 2. S2 & 6POS Switches Configuration Frame (Shifted to row 2)
        self.sw_frame = ttk.LabelFrame(parent, text=" Switches Pin & Type Configurations ")
        self.sw_frame.grid(row=2, column=0, columnspan=4, sticky="ew", padx=15, pady=5)

        # S2 switch
        ttk.Label(self.sw_frame, text="S2 RC Channel:").grid(row=0, column=0, sticky="e", padx=10, pady=5)
        self.vrx_s2_rc_var = tk.IntVar(value=8)
        self.ent_s2_rc = ttk.Entry(self.sw_frame, textvariable=self.vrx_s2_rc_var, width=8)
        self.ent_s2_rc.grid(row=0, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(self.sw_frame, text="S2 Switch Type:").grid(row=0, column=2, sticky="e", padx=10, pady=5)
        self.vrx_s2_type_var = tk.StringVar(value="8pos (Analog Dial)")
        self.combo_s2_type = ttk.Combobox(self.sw_frame, textvariable=self.vrx_s2_type_var, values=["2pos", "3pos", "6pos", "8pos (Analog Dial)"], width=15, state="readonly")
        self.combo_s2_type.grid(row=0, column=3, sticky="w", padx=10, pady=5)

        # 6POS switch
        ttk.Label(self.sw_frame, text="6POS RC Channel:").grid(row=1, column=0, sticky="e", padx=10, pady=5)
        self.vrx_6pos_rc_var = tk.IntVar(value=9)
        self.ent_6pos_rc = ttk.Entry(self.sw_frame, textvariable=self.vrx_6pos_rc_var, width=8)
        self.ent_6pos_rc.grid(row=1, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(self.sw_frame, text="6POS Switch Type:").grid(row=1, column=2, sticky="e", padx=10, pady=5)
        self.vrx_6pos_type_var = tk.StringVar(value="6pos")
        self.combo_6pos_type = ttk.Combobox(self.sw_frame, textvariable=self.vrx_6pos_type_var, values=["2pos", "3pos", "6pos"], width=15, state="readonly")
        self.combo_6pos_type.grid(row=1, column=3, sticky="w", padx=10, pady=5)

        # Table map channel (only active in mapping table mode)
        ttk.Label(self.sw_frame, text="Map Select RC Channel:").grid(row=2, column=0, sticky="e", padx=10, pady=5)
        self.vrx_rc_chan_var = tk.IntVar(value=8)
        self.ent_rc_chan = ttk.Entry(self.sw_frame, textvariable=self.vrx_rc_chan_var, width=8)
        self.ent_rc_chan.grid(row=2, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(self.sw_frame, text="Map Positions (2 to 8):").grid(row=2, column=2, sticky="e", padx=10, pady=5)
        self.vrx_pos_count_var = tk.IntVar(value=3)
        self.ent_pos_count = ttk.Spinbox(self.sw_frame, from_=2, to=8, textvariable=self.vrx_pos_count_var, width=8, command=self.update_switch_table_rows)
        self.ent_pos_count.grid(row=2, column=3, sticky="w", padx=10, pady=5)
        self.ent_pos_count.bind("<KeyRelease>", lambda e: self.update_switch_table_rows())

        # 3. Dynamic Positions Table mapping Frame (Only active in Custom Positions Mode - shifted to row 3)
        self.table_frame = ttk.LabelFrame(parent, text=" Switch Positions to Video Channel Mapping Table ")
        self.table_frame.grid(row=3, column=0, columnspan=4, sticky="nsew", padx=15, pady=5)

        ttk.Label(self.table_frame, text="RC Position", font=('Segoe UI', 9, 'bold')).grid(row=0, column=0, padx=15, pady=2)
        ttk.Label(self.table_frame, text="Target Band", font=('Segoe UI', 9, 'bold')).grid(row=0, column=1, padx=15, pady=2)
        ttk.Label(self.table_frame, text="Target Channel", font=('Segoe UI', 9, 'bold')).grid(row=0, column=2, padx=15, pady=2)
        ttk.Label(self.table_frame, text="Frequencies (MHz)", font=('Segoe UI', 9, 'bold')).grid(row=0, column=3, padx=15, pady=2)

        self.row_widgets = []
        for i in range(8):
            lbl_pos = ttk.Label(self.table_frame, text=f"Position {i+1}")
            lbl_pos.grid(row=i+1, column=0, padx=15, pady=2)

            band_var = tk.StringVar(value="Fatshark/F")
            combo_band = ttk.Combobox(self.table_frame, textvariable=band_var, values=BANDS_LIST, width=12, state="readonly")
            combo_band.grid(row=i+1, column=1, padx=15, pady=2)

            chan_var = tk.IntVar(value=1)
            combo_chan = ttk.Combobox(self.table_frame, textvariable=chan_var, values=[1, 2, 3, 4, 5, 6, 7, 8], width=8, state="readonly")
            combo_chan.grid(row=i+1, column=2, padx=15, pady=2)

            lbl_freq = ttk.Label(self.table_frame, text="5740 MHz")
            lbl_freq.grid(row=i+1, column=3, padx=15, pady=2)

            combo_band.bind("<<ComboboxSelected>>", lambda e, idx=i: self.on_table_row_changed(idx))
            combo_chan.bind("<<ComboboxSelected>>", lambda e, idx=i: self.on_table_row_changed(idx))

            self.row_widgets.append({
                'label_pos': lbl_pos,
                'band_var': band_var,
                'combo_band': combo_band,
                'chan_var': chan_var,
                'combo_chan': combo_chan,
                'lbl_freq': lbl_freq
            })

        for idx in range(8):
            self.on_table_row_changed(idx)

        self.update_switch_table_rows()
        self.on_vrx_control_mode_switched()

        self.btn_update_vrx = ttk.Button(parent, text="Upload Video Receiver Config", command=self.on_apply_vrx)
        self.btn_update_vrx.grid(row=4, column=0, columnspan=4, sticky="ew", padx=30, pady=10)

        # 4. Camera Switch & Pot Override
        div = ttk.Separator(parent, orient="horizontal")
        div.grid(row=5, column=0, columnspan=4, sticky="ew", pady=5)

        lbl_cam_head = ttk.Label(parent, text="Camera Switcher & Potentiometer Override Settings", style="Header.TLabel")
        lbl_cam_head.grid(row=6, column=0, columnspan=4, sticky="w", padx=20, pady=5)

        ttk.Label(parent, text="Cam Switch RC Channel:").grid(row=7, column=0, sticky="e", padx=10, pady=2)
        self.cam_rc_chan_var = tk.IntVar(value=7)
        self.ent_cam_rc_chan = ttk.Entry(parent, textvariable=self.cam_rc_chan_var, width=8)
        self.ent_cam_rc_chan.grid(row=7, column=1, sticky="w", padx=10, pady=2)

        ttk.Label(parent, text="Active Video Feed:").grid(row=7, column=2, sticky="e", padx=10, pady=2)
        self.active_cam_var = tk.StringVar(value="VRX Camera")
        self.cam_combo = ttk.Combobox(parent, textvariable=self.active_cam_var, values=["Analog Camera", "VRX Camera"], width=12, state="readonly")
        self.cam_combo.grid(row=7, column=3, sticky="w", padx=10, pady=2)

        self.pot_override_var = tk.BooleanVar(value=False)
        self.chk_pot_override = ttk.Checkbutton(parent, text="Manual Potentiometer Servo Control Override", variable=self.pot_override_var)
        self.chk_pot_override.grid(row=8, column=0, columnspan=4, sticky="w", padx=40, pady=5)

        self.btn_update_cam = ttk.Button(parent, text="Apply Camera & Potentiometer Settings", command=self.on_apply_cam)
        self.btn_update_cam.grid(row=9, column=0, columnspan=4, sticky="ew", padx=30, pady=5)

    # ------------------- Tab UI Helpers -------------------
    def on_vrx_control_mode_switched(self):
        mode_str = self.vrx_mode_var.get()

        # Clear selected mapping queue when switching modes to avoid unexpected highlight leakage
        self.selected_mapping_queue = []

        # 1. Enable/Disable S2 switch fields
        s2_state = "normal" if mode_str in ["S2 Only (Video Channel)", "S2 + 6POS (Simultaneous)"] else "disabled"
        self.ent_s2_rc.config(state=s2_state)
        self.combo_s2_type.config(state=s2_state)

        # 2. Enable/Disable 6POS switch fields
        p6_state = "normal" if mode_str in ["6POS Only (Video Band)", "S2 + 6POS (Simultaneous)"] else "disabled"
        self.ent_6pos_rc.config(state=p6_state)
        self.combo_6pos_type.config(state=p6_state)

        # 3. Enable/Disable custom table mapping fields
        tbl_state = "normal" if mode_str == "Custom Positions Table Mapping" else "disabled"
        self.ent_rc_chan.config(state=tbl_state)
        self.ent_pos_count.config(state=tbl_state)
        self.update_switch_table_rows()

        # Re-highlight grid cells accordingly
        self.highlight_active_freq_cell(self.conn.last_config.get('vrx_band', 0) if hasattr(self.conn, 'last_config') else 0,
                                        self.conn.last_config.get('vrx_chan', 0) if hasattr(self.conn, 'last_config') else 0)

    def update_switch_table_rows(self):
        mode_str = self.vrx_mode_var.get()
        if mode_str != "Custom Positions Table Mapping":
            for i in range(8):
                self.row_widgets[i]['combo_band'].config(state="disabled")
                self.row_widgets[i]['combo_chan'].config(state="disabled")
            return

        try:
            count = self.vrx_pos_count_var.get()
        except tk.TclError:
            count = 3
        count = max(2, min(count, 8))

        for i in range(8):
            state = "normal" if i < count else "disabled"
            self.row_widgets[i]['combo_band'].config(state=state)
            self.row_widgets[i]['combo_chan'].config(state=state)

    def on_table_row_changed(self, idx):
        band_str = self.row_widgets[idx]['band_var'].get()
        band_map = {
            "Band A": 0, "Band B": 1, "Band E": 2, "Fatshark/F": 3, "Raceband": 4,
            "Band D": 5, "Band X": 6, "Lowband/L": 7, "Band J": 8, "Band U": 9, "Band O": 10
        }
        band = band_map.get(band_str, 3)
        try:
            chan = int(self.row_widgets[idx]['chan_var'].get()) - 1
        except ValueError:
            chan = 0

        freq = VRX_FREQ_TABLE[band][chan]
        self.row_widgets[idx]['lbl_freq'].config(text=f"{freq} MHz")

    # ------------------- Action Callbacks -------------------
    def on_apply_switcher_settings(self):
        mode = self.mode_var.get()
        # Set switcher mode
        self.conn.set_mode(mode)
        # Apply complete VRX config (which appends switcher baudrates)
        self.on_apply_vrx()
        self.log(f"Switcher operational mode & baudrates updated: Mode={mode}")

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

            # Immediately move the map to center on the newly set home point coordinate setting!
            self.map_view.set_position(lat, lon)
            self.map_view.set_zoom(16)

            # Draw or update Ground Station Marker on map
            if self.home_marker:
                self.home_marker.set_position(lat, lon)
            else:
                # Use 10px circular icon programmatically generated to satisfy "reduce the antenna markers to 10 pixels"
                # Cascading fallback for tkintermapview version compatibility (handles both icon and image arguments)
                try:
                    self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station", icon=self.gs_dot_img)
                except TypeError:
                    try:
                        self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station", image=self.gs_dot_img)
                    except Exception:
                        self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station")
            self.last_known_home_pos = (lat, lon)
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
            positions = int(self.ent_pos_count.get())

            s2_rc = int(self.vrx_s2_rc_var.get())
            p6_rc = int(self.vrx_6pos_rc_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid values entered for VRX configuration.")
            return

        positions = max(2, min(positions, 8))

        # Get control mode
        m_str = self.vrx_mode_var.get()
        mode_map = {"S2 Only (Video Channel)": 1, "6POS Only (Video Band)": 2, "S2 + 6POS (Simultaneous)": 3, "Custom Positions Table Mapping": 4}
        vrx_control_mode = mode_map.get(m_str, 3)

        # Get S2 switch type
        s2_t_str = self.vrx_s2_type_var.get()
        s2_type_map = {"2pos": 2, "3pos": 3, "6pos": 6, "8pos (Analog Dial)": 8}
        s2_type = s2_type_map.get(s2_t_str, 8)

        # Get 6POS switch type
        p6_t_str = self.vrx_6pos_type_var.get()
        p6_type_map = {"2pos": 2, "3pos": 3, "6pos": 6}
        p6_type = p6_type_map.get(p6_t_str, 6)

        band_map = {
            "Band A": 0, "Band B": 1, "Band E": 2, "Fatshark/F": 3, "Raceband": 4,
            "Band D": 5, "Band X": 6, "Lowband/L": 7, "Band J": 8, "Band U": 9, "Band O": 10
        }

        mapped_list = []
        for i in range(8):
            b_str = self.row_widgets[i]['band_var'].get()
            band = band_map.get(b_str, 3)
            try:
                chan = int(self.row_widgets[i]['chan_var'].get()) - 1
            except ValueError:
                chan = 0
            mapped_list.append([band, chan])

        # Convert baudrates to divided-by-100 values
        try:
            b1 = int(self.jr1_crsf_baud_var.get()) // 100
            b2 = int(self.jr1_mav_baud_var.get()) // 100
            b3 = int(self.jr2_crsf_baud_var.get()) // 100
        except Exception:
            b1 = 4200
            b2 = 1152
            b3 = 4200

        # Extended Command 0x40 formatting:
        # [0x40, rc_chan, positions_count, vrx_control_mode, s2_rc, s2_type, p6_rc, p6_type, band_0, chan_0, ..., band_7, chan_7, jr1_crsf_h, jr1_crsf_l, jr1_mav_h, jr1_mav_l, jr2_crsf_h, jr2_crsf_l]
        payload = [0x40, rc_chan, positions, vrx_control_mode, s2_rc, s2_type, p6_rc, p6_type]
        for band, chan in mapped_list:
            payload.append(band)
            payload.append(chan)

        payload.append((b1 >> 8) & 0xFF)
        payload.append(b1 & 0xFF)
        payload.append((b2 >> 8) & 0xFF)
        payload.append(b2 & 0xFF)
        payload.append((b3 >> 8) & 0xFF)
        payload.append(b3 & 0xFF)

        if self.conn.send_command(payload):
            self.log(f"Extended VRX table & Baudrates applied: Mode={m_str}, JR1_CRSF={b1*100}, JR1_MAV={b2*100}, JR2_CRSF={b3*100}")
        else:
            self.log("Failed to apply video receiver & baudrates configuration.")

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
        self.root.after(0, self._on_config_received_main_thread, config)

    def _on_config_received_main_thread(self, config):
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

            # Draw or update Ground Station Marker on map
            lat, lon = config['home_lat'], config['home_lon']

            # If the home position has changed, center/relocate the map view to this location automatically!
            is_new_home = (self.last_known_home_pos is None or
                           abs(self.last_known_home_pos[0] - lat) > 1e-6 or
                           abs(self.last_known_home_pos[1] - lon) > 1e-6)

            if self.home_marker:
                self.home_marker.set_position(lat, lon)
            else:
                # Use 10px circular icon programmatically generated to satisfy "reduce the antenna markers to 10 pixels"
                # Cascading fallback for tkintermapview version compatibility (handles both icon and image arguments)
                try:
                    self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station", icon=self.gs_dot_img)
                except TypeError:
                    try:
                        self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station", image=self.gs_dot_img)
                    except Exception:
                        self.home_marker = self.map_view.set_marker(lat, lon, text="Ground Station")

            if is_new_home:
                self.map_view.set_position(lat, lon)
                self.map_view.set_zoom(16)

            self.last_known_home_pos = (lat, lon)

        # Draw and rotate the real-time antenna direction line on the map!
        if self.last_known_home_pos:
            self._update_antenna_direction_line(self.last_known_home_pos[0], self.last_known_home_pos[1], config['live_az'])

        # Update Cam Switch & Pot Overrides
        self.cam_rc_chan_var.set(config['cam_rc_chan'])
        self.active_cam_var.set("VRX Camera" if config['active_camera'] == 1 else "Analog Camera")
        self.pot_override_var.set(config['manual_override'] == 1)

        # Update tracking mode radio selection based on override
        t_mode = "manual" if config['manual_override'] == 1 else "auto"
        self.tracking_mode_var.set(t_mode)

        # Update live feedback display labels
        self.lbl_live_az.config(text=f"Live Azimuth: {config['live_az']}° (360° Limit)")
        self.lbl_live_el.config(text=f"Live Elevation: {config['live_el']}° (180° Limit)")

        # Update VRX extended settings
        self.ent_rc_chan.delete(0, tk.END)
        self.ent_rc_chan.insert(0, str(config['vrx_rc_chan']))
        self.vrx_pos_count_var.set(config['vrx_positions_count'])

        # Update the S2 & 6POS controls
        mode_map_rev = {1: "S2 Only (Video Channel)", 2: "6POS Only (Video Band)", 3: "S2 + 6POS (Simultaneous)", 4: "Custom Positions Table Mapping"}
        self.vrx_mode_var.set(mode_map_rev.get(config['vrx_control_mode'], "S2 + 6POS (Simultaneous)"))

        self.vrx_s2_rc_var.set(config['vrx_s2_rc_channel'])
        s2_type_rev = {2: "2pos", 3: "3pos", 6: "6pos", 8: "8pos (Analog Dial)"}
        self.vrx_s2_type_var.set(s2_type_rev.get(config['vrx_s2_switch_type'], "8pos (Analog Dial)"))

        self.vrx_6pos_rc_var.set(config['vrx_6pos_rc_channel'])
        p6_type_rev = {2: "2pos", 3: "3pos", 6: "6pos"}
        self.vrx_6pos_type_var.set(p6_type_rev.get(config['vrx_6pos_switch_type'], "6pos"))

        # Update switcher baudrates in the GUI
        if 'jr1_crsf_baud' in config:
            self.jr1_crsf_baud_var.set(str(config['jr1_crsf_baud'] * 100))
        if 'jr1_mav_baud' in config:
            self.jr1_mav_baud_var.set(str(config['jr1_mav_baud'] * 100))
        if 'jr2_crsf_baud' in config:
            self.jr2_crsf_baud_var.set(str(config['jr2_crsf_baud'] * 100))

        # Update the 8 mapping rows from the received board config
        band_map_rev = {
            0: "Band A", 1: "Band B", 2: "Band E", 3: "Fatshark/F", 4: "Raceband",
            5: "Band D", 6: "Band X", 7: "Lowband/L", 8: "Band J", 9: "Band U", 10: "Band O"
        }
        for i in range(8):
            b_val, c_val = config['vrx_mapped_channels'][i]
            self.row_widgets[i]['band_var'].set(band_map_rev.get(b_val, "Fatshark/F"))
            self.row_widgets[i]['chan_var'].set(c_val + 1)
            self.on_table_row_changed(i)

        self.update_switch_table_rows()
        self.on_vrx_control_mode_switched()

        # Highlight the current active video frequency cell in the grid
        self.highlight_active_freq_cell(config['vrx_band'], config['vrx_chan'])

        # Update connection status indicator labels
        if config.get('rf_board_online', 0) == 1:
            self.lbl_rf_status.config(text="RF: ONLINE", foreground='green')
        else:
            self.lbl_rf_status.config(text="RF: OFFLINE", foreground='red')

        if config.get('mavlink_active', 0) == 1:
            self.lbl_mav_status.config(text="MAV OK", foreground='green')
        else:
            self.lbl_mav_status.config(text="MAV: NO DATA", foreground='red')

        self.log(f"Stats Update: AZ={config['live_az']}°, EL={config['live_el']}°, Override={config['manual_override']}, Cam={'VRX' if config['active_camera'] == 1 else 'Analog'}, Switch_Pos={config['vrx_positions_count']}")

    def _periodic_heartbeat(self):
        if hasattr(self, 'conn') and self.conn and self.conn.running:
            # Silently request config status. This serves as a "PC Configurator Present" heartbeat
            # to prevent Board 1's 5-second GCS Raw fallback from triggering.
            self.conn.request_config_read()
            self.root.after(2000, self._periodic_heartbeat)

    def on_closing(self):
        self.conn.disconnect()
        self.root.destroy()

ZOOM = 1.0
