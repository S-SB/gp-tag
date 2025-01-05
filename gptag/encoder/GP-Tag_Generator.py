"""
MIT License

Copyright (c) 2025 S. E. Sundén Byléhn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import os
import sys
import math
from tag_encoder import create_fiducial_marker

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion using NED (North-East-Down) convention.
    
    In NED coordinates:
    - X axis points North
    - Y axis points East
    - Z axis points Down (through the tag's surface)
    
    At 0,0,0:
    - Tag lies flat on horizontal surface
    - Right side points North (+X)
    - Bottom points East (+Y)
    - Tag face is UP (visible when looking down)
    
    Args:
        roll (float): Rotation around X (North) axis in degrees
        pitch (float): Rotation around Y (East) axis in degrees
        yaw (float): Rotation around Z (Down) axis in degrees
    
    Returns:
        list: Quaternion [qx, qy, qz, qw]
    """
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return [qx, qy, qz, qw]

class GPTagGeneratorGUI:
    """
    A graphical user interface for generating GP-Tag fiducial markers.
    
    The GUI uses NED (North-East-Down) coordinate system for tag orientation:
    - X axis points North
    - Y axis points East
    - Z axis points Down (through the tag's surface)
    
    Default orientation (0,0,0):
    - Tag lies flat on horizontal surface
    - Right side points North (+X)
    - Bottom points East (+Y)
    - Tag face is UP (visible when looking down)
    
    Attributes:
        root (tk.Tk): The main window of the application
        latitude (tk.DoubleVar): Latitude in degrees (-90 to +90)
        longitude (tk.DoubleVar): Longitude in degrees (-180 to +180)
        altitude (tk.DoubleVar): Altitude in meters (-10000 to +10000)
        roll (tk.DoubleVar): Rotation around North axis (X) in degrees
        pitch (tk.DoubleVar): Rotation around East axis (Y) in degrees
        yaw (tk.DoubleVar): Rotation around Down axis (Z) in degrees
        scale (tk.DoubleVar): Physical tag size in millimeters (range: 10mm to 10000mm)
        accuracy (tk.IntVar): Position confidence level (0-3)
        tag_id (tk.IntVar): Unique tag identifier (0-4095)
        version_id (tk.IntVar): Version identifier (0-15)
        U (tk.IntVar): Base unit size in pixels
        save_path (tk.StringVar): Directory path for saving generated tags
        current_image (PIL.Image): Currently generated GP-Tag image
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("GP-Tag Generator")
        
        # Initialize variables with default values
        self.latitude = tk.DoubleVar(value=63.8203894)
        self.longitude = tk.DoubleVar(value=20.3058847)
        self.altitude = tk.DoubleVar(value=45.16)
        self.roll = tk.DoubleVar(value=0)
        self.pitch = tk.DoubleVar(value=0)
        self.yaw = tk.DoubleVar(value=0)
        self.scale = tk.DoubleVar(value=100)
        self.accuracy = tk.IntVar(value=2)
        self.tag_id = tk.IntVar(value=123)
        self.version_id = tk.IntVar(value=3)
        self.U = tk.IntVar(value=40)
        
        self.save_path = tk.StringVar(value=os.getcwd())
        self.current_image = None
        
        self.create_widgets()
        
    def create_widgets(self):
        """
        Create and arrange all GUI elements including input fields, preview area, and buttons.
        
        Creates interface sections:
        - NED coordinate system reference information
        - Global position inputs (latitude, longitude, altitude)
        - Tag orientation inputs (roll, pitch, yaw)
        - Tag parameters (scale, accuracy, IDs)
        - Tag preview canvas
        - Save controls
        """
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # NED Reference frame info
        ref_frame = ttk.LabelFrame(main_frame, text="Reference Frame (NED)", padding="5")
        ref_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=5, pady=5)
        
        ref_text = """Default Orientation (0,0,0):
• Tag lies flat on horizontal surface
• Right side points North (+X)
• Bottom points East (+Y)
• Tag face is UP (visible when looking down)"""
        ttk.Label(ref_frame, text=ref_text, justify=tk.LEFT).grid(row=0, column=0, padx=5, pady=5)
        
        # Input parameters
        input_frame = ttk.LabelFrame(main_frame, text="Tag Parameters", padding="5")
        input_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)
        
        # Position inputs
        ttk.Label(input_frame, text="Global Position:").grid(row=0, column=0, sticky=tk.W)
        
        lat_frame = ttk.Frame(input_frame)
        lat_frame.grid(row=1, column=0)
        lat_entry = ttk.Entry(lat_frame, textvariable=self.latitude, width=15)
        lat_entry.grid(row=0, column=0)
        ttk.Label(lat_frame, text="Latitude").grid(row=1, column=0)
        self.create_tooltip(lat_entry, "Latitude in degrees\nRange: -90° (South) to +90° (North)")
        
        lon_frame = ttk.Frame(input_frame)
        lon_frame.grid(row=1, column=1)
        lon_entry = ttk.Entry(lon_frame, textvariable=self.longitude, width=15)
        lon_entry.grid(row=0, column=0)
        ttk.Label(lon_frame, text="Longitude").grid(row=1, column=0)
        self.create_tooltip(lon_entry, "Longitude in degrees\nRange: -180° (West) to +180° (East)")
        
        alt_frame = ttk.Frame(input_frame)
        alt_frame.grid(row=1, column=2)
        alt_entry = ttk.Entry(alt_frame, textvariable=self.altitude, width=15)
        alt_entry.grid(row=0, column=0)
        ttk.Label(alt_frame, text="Altitude").grid(row=1, column=0)
        self.create_tooltip(alt_entry, "Altitude in meters\nRange: -10000m to +10000m")
        
        # Orientation inputs
        ttk.Label(input_frame, text="Tag Orientation (Euler Angles):").grid(row=3, column=0, sticky=tk.W, pady=(10,0))
        
        roll_frame = ttk.Frame(input_frame)
        roll_frame.grid(row=4, column=0)
        roll_entry = ttk.Entry(roll_frame, textvariable=self.roll, width=15)
        roll_entry.grid(row=0, column=0)
        ttk.Label(roll_frame, text="Roll (X°)").grid(row=1, column=0)
        self.create_tooltip(roll_entry, "Rotation around North axis (X)\nPositive: Tag rotates clockwise\nviewed from North")
        
        pitch_frame = ttk.Frame(input_frame)
        pitch_frame.grid(row=4, column=1)
        pitch_entry = ttk.Entry(pitch_frame, textvariable=self.pitch, width=15)
        pitch_entry.grid(row=0, column=0)
        ttk.Label(pitch_frame, text="Pitch (Y°)").grid(row=1, column=0)
        self.create_tooltip(pitch_entry, "Rotation around East axis (Y)\nPositive: Tag rotates clockwise\nviewed from East")
        
        yaw_frame = ttk.Frame(input_frame)
        yaw_frame.grid(row=4, column=2)
        yaw_entry = ttk.Entry(yaw_frame, textvariable=self.yaw, width=15)
        yaw_entry.grid(row=0, column=0)
        ttk.Label(yaw_frame, text="Yaw (Z°)").grid(row=1, column=0)
        self.create_tooltip(yaw_entry, "Rotation around Down axis (Z)\nPositive: Tag rotates clockwise\nviewed from above")
        
        # Other parameters
        ttk.Label(input_frame, text="Tag Parameters:").grid(row=6, column=0, sticky=tk.W, pady=(10,0))
        
        # Tag Size and Accuracy row
        tag_size_frame = ttk.Frame(input_frame)
        tag_size_frame.grid(row=7, column=0)
        tag_size_entry = ttk.Entry(tag_size_frame, textvariable=self.scale, width=15)
        tag_size_entry.grid(row=0, column=0)
        ttk.Label(tag_size_frame, text="Tag Size (mm)").grid(row=1, column=0)
        self.create_tooltip(tag_size_entry, "Physical tag size in millimeters\nRange: 10mm to 10000mm (1cm to 10m)\nLarger tags improve detection range")

        acc_frame = ttk.Frame(input_frame)
        acc_frame.grid(row=7, column=1)
        acc_entry = ttk.Entry(acc_frame, textvariable=self.accuracy, width=15)
        acc_entry.grid(row=0, column=0)
        ttk.Label(acc_frame, text="Accuracy (0-3)").grid(row=1, column=0)
        self.create_tooltip(acc_entry, "Position confidence level\n0: Low accuracy (>10m)\n1: Medium accuracy (1-10m)\n2: High accuracy (0.1-1m)\n3: Ultra-high accuracy (<0.1m)")

        # IDs row
        tag_frame = ttk.Frame(input_frame)
        tag_frame.grid(row=8, column=0)
        tag_entry = ttk.Entry(tag_frame, textvariable=self.tag_id, width=15)
        tag_entry.grid(row=0, column=0)
        ttk.Label(tag_frame, text="Tag ID (0-4095)").grid(row=1, column=0)
        
        ver_frame = ttk.Frame(input_frame)
        ver_frame.grid(row=8, column=1)
        ver_entry = ttk.Entry(ver_frame, textvariable=self.version_id, width=15)
        ver_entry.grid(row=0, column=0)
        ttk.Label(ver_frame, text="Version (0-15)").grid(row=1, column=0)
        
        # Resolution row
        res_frame = ttk.Frame(input_frame)
        res_frame.grid(row=9, column=0)
        res_entry = ttk.Entry(res_frame, textvariable=self.U, width=15)
        res_entry.grid(row=0, column=0)
        ttk.Label(res_frame, text="Resolution (U)").grid(row=1, column=0)
        self.create_tooltip(res_entry, "Pixels per cell in output image\nFinal tag size will be 36U × 36U pixels\nExample: U=10 gives 360×360px tag")
        
        ttk.Button(input_frame, text="Generate Tag", command=self.generate_tag).grid(row=11, column=0, columnspan=4, pady=10)
        
        # Preview area
        display_frame = ttk.LabelFrame(main_frame, text="Tag Preview", padding="5")
        display_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5)
        
        self.canvas = tk.Canvas(display_frame, width=400, height=400, bg='white')
        self.canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Save controls
        save_frame = ttk.Frame(main_frame, padding="5")
        save_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Entry(save_frame, textvariable=self.save_path, width=50).grid(row=0, column=0, padx=5)
        ttk.Button(save_frame, text="Browse", command=self.browse_save_location).grid(row=0, column=1, padx=5)
        ttk.Button(save_frame, text="Save Tag", command=self.save_tag).grid(row=0, column=2, padx=5)

    def create_tooltip(self, widget, text):
        """
        Create a hover tooltip for a given widget.
        
        Args:
            widget: The tkinter widget to attach tooltip to
            text (str): The tooltip text to display
        """
        def show_tooltip(event):
            tooltip = tk.Toplevel()
            tooltip.wm_overrideredirect(True)
            tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
            
            label = ttk.Label(tooltip, text=text, justify=tk.LEFT,
                            background="#ffffe0", relief="solid", borderwidth=1)
            label.pack()
            
            def hide_tooltip():
                tooltip.destroy()
            
            widget.tooltip = tooltip
            widget.bind('<Leave>', lambda e: hide_tooltip())
            tooltip.bind('<Leave>', lambda e: hide_tooltip())
        
        widget.bind('<Enter>', show_tooltip)

    def generate_tag(self):
        """
        Generate a GP-Tag with current parameter values.
        
        Converts Euler angles to quaternion, generates the tag image,
        and displays preview in the GUI canvas.
        
        Handles errors with message dialogs.
        """
        try:
            quaternion = euler_to_quaternion(
                self.roll.get(),
                self.pitch.get(),
                self.yaw.get()
            )
            # Tag size in mm -> scale cells/mm
            tag_size_mm = self.scale.get()  
            calculated_scale = 36 / tag_size_mm 

            self.current_image = create_fiducial_marker(
                self.latitude.get(),
                self.longitude.get(),
                self.altitude.get(),
                quaternion,
                calculated_scale,
                self.accuracy.get(),
                self.tag_id.get(),
                self.version_id.get(),
                U=self.U.get()
            )
            
            if self.current_image:
                display_size = (400, 400)
                display_image = self.current_image.copy()
                display_image.thumbnail(display_size, Image.Resampling.LANCZOS)
                
                self.photo = ImageTk.PhotoImage(display_image)
                self.canvas.create_image(
                    200, 200,
                    image=self.photo,
                    anchor=tk.CENTER
                )
            else:
                messagebox.showerror("Error", "Failed to generate tag")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error generating tag: {str(e)}")

    def browse_save_location(self):
        """Open file dialog for selecting save directory."""
        directory = filedialog.askdirectory(initialdir=self.save_path.get())
        if directory:
            self.save_path.set(directory)

    def save_tag(self):
        """
        Save the currently generated tag to a PNG file.
        
        Generates filename using tag_id and handles save errors
        with message dialogs.
        """
        if not self.current_image:
            messagebox.showwarning("Warning", "Generate a tag first!")
            return
            
        try:
            filename = f"gptag_{self.tag_id.get()}.png"
            save_path = os.path.join(self.save_path.get(), filename)
            self.current_image.save(save_path)
            messagebox.showinfo("Success", f"Tag saved as {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Error saving tag: {str(e)}")


def main():
    """Initialize and run the GP-Tag Generator application."""
    root = tk.Tk()
    app = GPTagGeneratorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()