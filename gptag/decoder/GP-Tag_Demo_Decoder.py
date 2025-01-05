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

import cv2
import numpy as np
from pathlib import Path
from typing import List, Optional, Dict, Tuple, Union
from sift_detector import SIFTDetector6DoF

def create_visualization(original_image: np.ndarray, 
                        corners: List[List[float]], 
                        rectified_image: np.ndarray,
                        window_name: str = "GP-Tag Detection Results") -> np.ndarray:
    """
    Create a side-by-side visualization of GP-Tag detection results.
    
    Creates a combined visualization showing:
    1. Original image with detected tag boundary and numbered corners
    2. Rectified tag image after perspective correction
    
    Both images are scaled to the same height and labeled for clarity.
    
    Args:
        original_image: Input image where tag was detected
        corners: List of [x,y] corner coordinates from detection
        rectified_image: Perspective-corrected tag image
        window_name: Title for the visualization window
        
    Returns:
        Combined visualization image showing both views side by side
        
    Notes:
        - Original image shows corners numbered 0-3 in clockwise order
        - Green boundary box shows detected tag perimeter
        - Blue dots mark exact corner positions
        - Both images are labeled for clear identification
    """
    # Create copy of original image for drawing
    detection_vis = original_image.copy()
    
    # Convert corners to integer points and numpy array
    corners = np.array(corners, dtype=np.int32)
    
    # Draw boundary box
    cv2.polylines(detection_vis, [corners], True, (0, 255, 0), 2)
    
    # Add corner numbers (clockwise from top-left)
    for i, corner in enumerate(corners):
        cv2.circle(detection_vis, tuple(corner), 5, (255, 0, 0), -1)
        cv2.putText(detection_vis, str(i), tuple(corner + 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    
    # Convert rectified image to BGR if it's grayscale
    if len(rectified_image.shape) == 2:
        rectified_vis = cv2.cvtColor(rectified_image, cv2.COLOR_GRAY2BGR)
    else:
        rectified_vis = rectified_image.copy()
    
    # Resize rectified image to match height of original
    scale_factor = detection_vis.shape[0] / rectified_vis.shape[0]
    new_width = int(rectified_vis.shape[1] * scale_factor)
    rectified_vis = cv2.resize(rectified_vis, (new_width, detection_vis.shape[0]))
    
    # Add labels
    cv2.putText(detection_vis, "Original Detection", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(rectified_vis, "Rectified Tag", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Combine images horizontally
    combined_vis = np.hstack([detection_vis, rectified_vis])
    
    return combined_vis

def quaternion_to_euler_NED(q: List[float]) -> List[float]:
    """
    Convert quaternion to Euler angles following NED (North-East-Down) convention.
    
    Converts a quaternion in [x,y,z,w] format to Euler angles [roll, pitch, yaw]
    in degrees. Uses the NED frame convention where pitch is negated compared to
    the standard aerospace sequence.
    
    In NED frame:
    - Roll: Rotation around North axis (X)
    - Pitch: Rotation around East axis (Y), negated
    - Yaw: Rotation around Down axis (Z)
    
    Reference pose (0,0,0):
    - Tag lying flat on ground (Z down)
    - Tag's right side pointing North
    - Tag's top pointing East
    
    Args:
        q: Quaternion as [x,y,z,w]
        
    Returns:
        List of Euler angles [roll, pitch, yaw] in degrees, NED convention
        
    Notes:
        - Angles are returned in degrees
        - Pitch is negated to follow NED convention
        - Gimbal lock is handled for pitch near ±90°
    """
    x, y, z, w = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [
        np.degrees(roll),
        np.degrees(-pitch),  # Negate pitch for NED frame
        np.degrees(yaw)
    ]

def calculate_observer_position(tag_position: List[float], 
                              tag_lat: float, 
                              tag_lon: float, 
                              tag_alt: float) -> Tuple[float, float, float]:
    """
    Calculate observer's global position based on tag position and pose in NED frame.
    
    Uses basic geodetic calculations to determine the observer's position given
    the tag's global position and the relative position vector to the observer
    in the NED frame.
    
    Args:
        tag_position: [north, east, down] position vector in meters
        tag_lat: Tag latitude in degrees
        tag_lon: Tag longitude in degrees
        tag_alt: Tag altitude in meters
        
    Returns:
        Tuple of (latitude, longitude, altitude) for observer position
        
    Notes:
        - Uses WGS84 Earth radius for calculations
        - Assumes locally flat Earth approximation for small distances
        - Position vector is in NED frame where:
            - X axis points North
            - Y axis points East
            - Z axis points Down
        - Calculations are approximate for small distances (<10km)
    """
    # WGS84 equatorial radius
    EARTH_RADIUS = 6378137.0  # meters
    
    # Extract NED components
    n, e, d = tag_position
    
    # Calculate changes in latitude and longitude
    lat_change = np.degrees(n / EARTH_RADIUS)
    lon_change = np.degrees(e / (EARTH_RADIUS * np.cos(np.radians(tag_lat))))
    
    # Calculate observer position
    obs_lat = tag_lat - lat_change  # Subtract because tag position is relative to observer
    obs_lon = tag_lon - lon_change
    obs_alt = tag_alt + d  # Add because NED z is down
    
    return obs_lat, obs_lon, obs_alt

def print_detection_results(results: Dict) -> None:
    """
    Print GP-Tag detection results in a human-readable format using NED frame conventions.
    
    Formats and displays the complete detection results including:
    - Tag metadata and ID
    - Global position data
    - Tag and camera orientation in both quaternion and Euler angles
    - Observer's calculated global position
    - Detection timing statistics
    
    All orientations follow the NED (North-East-Down) frame convention where:
    - Reference pose (0,0,0) is tag flat on ground with right side facing north
    - Pitch angles are negated compared to standard aerospace sequence
    - Position vectors are expressed as [north, east, down]
    
    Args:
        results: Detection results dictionary from SIFTDetector6DoF.detect()
            Must contain position, rotation, and tag_data if successful
            
    Notes:
        - All angles are displayed in degrees
        - Positions are in meters
        - NED frame is used consistently throughout
    """
    print("\nGP-Tag Detection Results:")
    print("-" * 50)
    print("Using NED (North-East-Down) coordinate frame")
    print("Reference pose: tag flat on ground (z-down), right side facing north")
    
    if not results:
        print("No tag detected.")
        return
        
    print(f"Detection Time: {results['detection_time_ms']:.1f}ms")
    
    if results.get('tag_data'):
        tag_data = results['tag_data']
        print("\nTag Data:")
        print(f"  Tag ID: {tag_data['tag_id']}")
        print(f"  Version: {tag_data['version_id']}")
        
        print(f"\nTag Global Position (NED frame):")
        print(f"  Latitude:  {tag_data['latitude']:.6f}°")
        print(f"  Longitude: {tag_data['longitude']:.6f}°")
        print(f"  Altitude:  {tag_data['altitude']:.1f}m")
        
        print("\nTag Orientation (NED frame):")
        print("  Quaternion [x,y,z,w]:")
        print(f"    [{', '.join([f'{x:.3f}' for x in tag_data['quaternion']])}]")
        
        euler = quaternion_to_euler_NED(tag_data['quaternion'])
        print("  Euler angles [roll, pitch, yaw] (degrees):")
        print(f"    [{', '.join([f'{x:.1f}°' for x in euler])}]")
        print("    (pitch is negative in NED frame)")
        
        print(f"\nTag Metadata:")
        print(f"  Accuracy:  Level {tag_data['accuracy']}")
        print(f"  Scale:     {tag_data['scale']:.3f} cells/mm")
        
    print("\nCamera-Tag Relative Pose (NED frame):")
    print(f"  Position [north, east, down]:")
    print(f"    [{', '.join([f'{x:.3f}m' for x in results['position']])}]")
    print(f"  Rotation quaternion [x,y,z,w]:")
    print(f"    [{', '.join([f'{x:.3f}' for x in results['rotation']])}]")
    
    cam_euler = quaternion_to_euler_NED(results['rotation'])
    print("  Rotation Euler [roll, pitch, yaw] (degrees):")
    print(f"    [{', '.join([f'{x:.1f}°' for x in cam_euler])}]")
    print("    (pitch is negative in NED frame)")
    
    if results.get('tag_data'):
        obs_lat, obs_lon, obs_alt = calculate_observer_position(
            results['position'],
            tag_data['latitude'],
            tag_data['longitude'],
            tag_data['altitude']
        )
        print("\nObserver Global Position:")
        print(f"  Latitude:  {obs_lat:.6f}°")
        print(f"  Longitude: {obs_lon:.6f}°")
        print(f"  Altitude:  {obs_alt:.1f}m")
    
    if results.get('timing_stats'):
        print("\nTiming Breakdown:")
        for stage, time in results['timing_stats'].items():
            print(f"  {stage}: {time:.1f}ms")

def main() -> None:
    """
    Run a demonstration of GP-Tag detection on a sample image.
    
    This demo:
    1. Loads an example GP-Tag image
    2. Sets up camera parameters (calibrated for 1920x1080 resolution)
    3. Initializes and runs the GP-Tag detector
    4. Displays comprehensive results including:
       - Global position data (lat, lon, alt)
       - 6-DoF pose estimation in NED frame
       - Tag and camera orientations
       - Calculated observer position
       - Detection timing information
       - Visual results showing original detection and rectified tag
    
    Required files (must be in same directory):
    - tag3_example_1117.png: Example image containing a GP-Tag
    - tag3_blank_360.png: Template image for SIFT matching
    
    Notes:
        - Camera parameters are configured for 1920x1080 resolution
        - Adjust camera_matrix values for your specific camera
        - All results use NED (North-East-Down) coordinate frame
        - Press any key to exit visualization window
    """
    # Load example image
    image_path = Path(__file__).parent / "tag3_example_1117.png"
    if not image_path.exists():
        print(f"Error: Example image not found at {image_path}")
        return
        
    image = cv2.imread(str(image_path))
    if image is None:
        print("Error: Failed to load image")
        return

    # Camera parameters (calibrated for 1920x1080 camera)
    camera_matrix = np.array([
        [961.267, 0, 964.3],
        [0, 961.267, 538.868],
        [0, 0, 1]
    ])
    dist_coeffs = np.zeros(5)  # Assuming no distortion for demo

    # Initialize detector
    detector = SIFTDetector6DoF()

    try:
        # Run detection
        results = detector.detect(
            image=image,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            debug_info=False,
            save_imgs=False
        )

        # Print results
        print_detection_results(results)

        # Create and show visualization
        if results and 'rectified_image' in results and 'corners' in results:
            # Create combined visualization
            vis_image = create_visualization(
                original_image=image,
                corners=results['corners'],
                rectified_image=results['rectified_image']
            )
            
            # Show visualization window
            window_name = "GP-Tag Detection Results"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, vis_image)
            print("\nPress any key to exit...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error during detection: {str(e)}")

if __name__ == "__main__":
    main()