import tkinter as tk
from gui import ConfiguratorApp

def main():
    root = tk.Tk()
    app = ConfiguratorApp(root)

    # Run the Tkinter main loop
    # Starts the beautiful visual GUI interface for configuration, calibration,
    # mode switching, active telemetry display, and camera toggles.
    root.mainloop()

if __name__ == "__main__":
    main()
