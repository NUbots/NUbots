#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import argparse
import cv2
import numpy as np
import os


def register(command):
    command.description = "Generate ArUco marker PNG image for printing on A4 paper"

    # Command arguments
    command.add_argument("--aruco-id", type=int, required=True, help="ID of the ArUco marker to generate")
    command.add_argument("--aruco-dict", default="DICT_6X6_250", help="ArUco dictionary to use")
    command.add_argument("--marker-size-mm", type=float, default=100.0, help="Size of the marker in millimeters")
    command.add_argument("--margin-mm", type=float, default=20.0, help="Margin around the marker in millimeters")
    command.add_argument("--dpi", type=int, default=300, help="Output image DPI (dots per inch)")
    command.add_argument("--output", "-o", default="aruco_marker.png", help="Output PNG filename")
    command.add_argument("--add-info", action="store_true", help="Add marker information text to the image")


def mm_to_pixels(mm, dpi):
    """Convert millimeters to pixels at given DPI"""
    inches = mm / 25.4
    return int(inches * dpi)


def create_aruco_marker_image(aruco_id, aruco_dict, marker_size_mm, margin_mm, dpi, add_info=False):
    """Create an ArUco marker image suitable for printing"""

    # Convert dimensions to pixels
    marker_size_px = mm_to_pixels(marker_size_mm, dpi)
    margin_px = mm_to_pixels(margin_mm, dpi)

    # Generate the ArUco marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, aruco_id, marker_size_px)

    # Add margins around the marker
    total_size = marker_size_px + 2 * margin_px
    output_img = np.ones((total_size, total_size), dtype=np.uint8) * 255  # White background

    # Place the marker in the center
    y_offset = margin_px
    x_offset = margin_px
    output_img[y_offset:y_offset+marker_size_px, x_offset:x_offset+marker_size_px] = marker_img

    # Add information text if requested
    if add_info:
        # Convert to RGB for colored text
        output_img_rgb = cv2.cvtColor(output_img, cv2.COLOR_GRAY2RGB)

        # Text information
        info_lines = [
            f"ArUco ID: {aruco_id}",
            f"Dictionary: {aruco_dict}",
            f"Marker Size: {marker_size_mm}mm",
            f"Print at {dpi} DPI"
        ]

        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        font_thickness = 2
        text_color = (0, 0, 0)  # Black text
        line_spacing = 30

        # Add text below the marker
        text_y = total_size - margin_px + 20
        for i, line in enumerate(info_lines):
            text_x = margin_px
            text_y_pos = text_y + i * line_spacing
            cv2.putText(output_img_rgb, line, (text_x, text_y_pos),
                       font, font_scale, text_color, font_thickness)

        return output_img_rgb

    return output_img


def create_a4_sheet_with_marker(aruco_id, aruco_dict, marker_size_mm, margin_mm, dpi, add_info=False):
    """Create an A4 sheet with the ArUco marker centered"""

    # A4 dimensions in mm: 210 x 297
    a4_width_mm = 210
    a4_height_mm = 297

    # Convert to pixels
    a4_width_px = mm_to_pixels(a4_width_mm, dpi)
    a4_height_px = mm_to_pixels(a4_height_mm, dpi)

    # Create A4-sized white background
    a4_img = np.ones((a4_height_px, a4_width_px), dtype=np.uint8) * 255

    # Generate the marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, aruco_id, mm_to_pixels(marker_size_mm, dpi))
    marker_size_px = marker_img.shape[0]

    # Calculate center position
    center_x = a4_width_px // 2
    center_y = a4_height_px // 2

    # Place marker in center
    x_offset = center_x - marker_size_px // 2
    y_offset = center_y - marker_size_px // 2

    a4_img[y_offset:y_offset+marker_size_px, x_offset:x_offset+marker_size_px] = marker_img

    # Add information text if requested
    if add_info:
        # Convert to RGB for colored text
        a4_img_rgb = cv2.cvtColor(a4_img, cv2.COLOR_GRAY2RGB)

        # Text information
        info_lines = [
            f"ArUco ID: {aruco_id}",
            f"Dictionary: {aruco_dict}",
            f"Marker Size: {marker_size_mm}mm",
            f"Print at {dpi} DPI on A4 paper"
        ]

        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        font_thickness = 3
        text_color = (0, 0, 0)  # Black text
        line_spacing = 40

        # Add text below the marker
        text_y = center_y + marker_size_px // 2 + 50
        for i, line in enumerate(info_lines):
            text_size = cv2.getTextSize(line, font, font_scale, font_thickness)[0]
            text_x = center_x - text_size[0] // 2
            text_y_pos = text_y + i * line_spacing
            cv2.putText(a4_img_rgb, line, (text_x, text_y_pos),
                       font, font_scale, text_color, font_thickness)

        return a4_img_rgb

    return a4_img


def run(aruco_id, aruco_dict, marker_size_mm, margin_mm, dpi, output, add_info, **kwargs):
    """Generate ArUco marker image"""

    # Get ArUco dictionary
    aruco_dict_id = getattr(cv2.aruco, aruco_dict)
    aruco_dict_obj = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    print(f"Generating ArUco marker:")
    print(f"  ID: {aruco_id}")
    print(f"  Dictionary: {aruco_dict}")
    print(f"  Marker size: {marker_size_mm}mm")
    print(f"  Margin: {margin_mm}mm")
    print(f"  DPI: {dpi}")
    print(f"  Output: {output}")
    print(f"  Add info: {add_info}")

    # Check if marker ID is valid for the dictionary
    max_id = aruco_dict_obj.bytesList.shape[0] - 1
    if aruco_id > max_id:
        print(f"Error: Marker ID {aruco_id} is too large for dictionary {aruco_dict}")
        print(f"Maximum ID for this dictionary is {max_id}")
        return

    # Create the marker image
    if add_info:
        # Create A4 sheet with centered marker and info
        marker_img = create_a4_sheet_with_marker(
            aruco_id, aruco_dict_obj, marker_size_mm, margin_mm, dpi, add_info
        )
    else:
        # Create simple marker with margins
        marker_img = create_aruco_marker_image(
            aruco_id, aruco_dict_obj, marker_size_mm, margin_mm, dpi, add_info
        )

    # Save the image
    success = cv2.imwrite(output, marker_img)

    if success:
        print(f"\n✓ Successfully generated ArUco marker image: {output}")

        # Print dimensions
        height, width = marker_img.shape[:2]
        print(f"  Image dimensions: {width} x {height} pixels")

        # Calculate physical dimensions
        width_mm = width * 25.4 / dpi
        height_mm = height * 25.4 / dpi
        print(f"  Physical dimensions: {width_mm:.1f} x {height_mm:.1f} mm")

        if add_info:
            print(f"  Ready for printing on A4 paper at {dpi} DPI")
        else:
            print(f"  Ready for printing at {dpi} DPI")

    else:
        print(f"\n✗ Failed to save image: {output}")


if __name__ == "__main__":
    # For direct script execution
    parser = argparse.ArgumentParser(description="Generate ArUco marker PNG image for printing")
    parser.add_argument("--aruco-id", type=int, required=True, help="ID of the ArUco marker to generate")
    parser.add_argument("--aruco-dict", default="DICT_6X6_250", help="ArUco dictionary to use")
    parser.add_argument("--marker-size-mm", type=float, default=100.0, help="Size of the marker in millimeters")
    parser.add_argument("--margin-mm", type=float, default=20.0, help="Margin around the marker in millimeters")
    parser.add_argument("--dpi", type=int, default=300, help="Output image DPI (dots per inch)")
    parser.add_argument("--output", "-o", default="aruco_marker.png", help="Output PNG filename")
    parser.add_argument("--add-info", action="store_true", help="Add marker information text to the image")

    args = parser.parse_args()
    run(**vars(args))
