#!/usr/bin/env python3
"""
Generate ArUco marker images for printing
"""

import argparse

import cv2
import numpy as np


def generate_aruco_marker(marker_id, dict_name, marker_size_mm, dpi=300, border_size_mm=10):
    """
    Generate an ArUco marker image suitable for printing

    Args:
        marker_id: ID of the ArUco marker to generate
        dict_name: Name of ArUco dictionary (e.g., 'DICT_6X6_250')
        marker_size_mm: Size of the marker in millimeters
        dpi: Dots per inch for printing (300 is good for most printers)
        border_size_mm: White border around the marker in millimeters
    """

    # Get the ArUco dictionary - corrected approach
    aruco_dict_id = getattr(cv2.aruco, dict_name)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    # Calculate pixel dimensions
    # Convert mm to inches, then to pixels
    marker_size_inches = marker_size_mm / 25.4
    marker_size_pixels = int(marker_size_inches * dpi)

    border_size_inches = border_size_mm / 25.4
    border_size_pixels = int(border_size_inches * dpi)

    # Generate the marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels)

    # Add white border
    total_size = marker_size_pixels + 2 * border_size_pixels
    bordered_img = np.ones((total_size, total_size), dtype=np.uint8) * 255  # White background

    # Place marker in center
    start_pos = border_size_pixels
    end_pos = start_pos + marker_size_pixels
    bordered_img[start_pos:end_pos, start_pos:end_pos] = marker_img

    return bordered_img, total_size, dpi


def main():
    parser = argparse.ArgumentParser(description='Generate ArUco marker for printing')
    parser.add_argument('--marker-id', type=int, default=0, help='ArUco marker ID')
    parser.add_argument('--dict', default='DICT_6X6_250', help='ArUco dictionary')
    parser.add_argument('--size-mm', type=float, default=100, help='Marker size in millimeters')
    parser.add_argument('--border-mm', type=float, default=10, help='Border size in millimeters')
    parser.add_argument('--dpi', type=int, default=300, help='DPI for printing')
    parser.add_argument('--output', default='aruco_marker.png', help='Output filename')

    args = parser.parse_args()

    print(f"Generating ArUco marker:")
    print(f"  ID: {args.marker_id}")
    print(f"  Dictionary: {args.dict}")
    print(f"  Size: {args.size_mm}mm")
    print(f"  Border: {args.border_mm}mm")
    print(f"  DPI: {args.dpi}")

    try:
        # Generate the marker
        marker_img, total_size_pixels, dpi = generate_aruco_marker(
            args.marker_id, args.dict, args.size_mm, args.dpi, args.border_mm
        )

        # Save the image
        cv2.imwrite(args.output, marker_img)

        # Calculate final dimensions
        total_size_mm = args.size_mm + 2 * args.border_mm
        total_size_inches = total_size_mm / 25.4

        print(f"\nMarker generated successfully!")
        print(f"  Output file: {args.output}")
        print(f"  Image size: {total_size_pixels}x{total_size_pixels} pixels")
        print(f"  Physical size: {total_size_mm:.1f}mm x {total_size_mm:.1f}mm")
        print(f"  Physical size: {total_size_inches:.2f}\" x {total_size_inches:.2f}\"")

        # Check if it fits on A4
        a4_width_mm = 210
        a4_height_mm = 297

        if total_size_mm <= min(a4_width_mm, a4_height_mm):
            print(f"✓ Will fit on A4 paper ({a4_width_mm}mm x {a4_height_mm}mm)")
        else:
            print(f"⚠ May not fit on A4 paper ({a4_width_mm}mm x {a4_height_mm}mm)")
            print(f"  Consider reducing --size-mm or --border-mm")

        print(f"\nPrinting instructions:")
        print(f"1. Open {args.output} in an image viewer or document editor")
        print(f"2. Print at 100% scale (no scaling/fitting)")
        print(f"3. Verify the printed size with a ruler")
        print(f"4. The marker should measure {args.size_mm}mm x {args.size_mm}mm")

    except Exception as e:
        print(f"Error generating marker: {e}")
        print(f"Available dictionaries:")
        # List some common ArUco dictionaries
        common_dicts = [
            'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
            'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
            'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
            'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000'
        ]
        for dict_name in common_dicts:
            if hasattr(cv2.aruco, dict_name):
                print(f"  {dict_name}")


if __name__ == "__main__":
    main()
