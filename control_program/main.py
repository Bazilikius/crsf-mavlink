import tkinter as tk
from gui import ConfiguratorApp

def main():
    root = tk.Tk()
    app = ConfiguratorApp(root)

    # Run the Tkinter main loop
    # Will start the beautiful visual interface for configuration, calibration,
    # mode switching and monitoring.
    root.mainloop()

if __name__ == "__main__":
    main()
