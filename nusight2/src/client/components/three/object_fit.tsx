import React from "react";
import { useThree } from "@react-three/fiber";

/**
 * Component scales children so that an object with the given width and height is as large
 * as possible while being contained by the canvas and maintaining its aspect ratio.
 *
 * Based on CSS 'object-fit' property.
 */
export function ObjectFitContain(props: { width: number; height: number; children: React.ReactNode }) {
  const { width, height, children } = props;
  const canvas = useThree((state) => state.size);

  const objectAspectRatio = width / height;
  const canvasAspectRatio = canvas.width / canvas.height;

  // Depending on which has a wider aspect ratio, scale using the width ratio or height ratio
  const scale = objectAspectRatio > canvasAspectRatio ? canvas.width / width : canvas.height / height;

  return <group scale={scale}>{children}</group>;
}
