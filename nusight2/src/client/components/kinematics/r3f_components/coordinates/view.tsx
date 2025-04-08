import React, { useEffect, useRef, useState } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";

export const CoordinateLabel = ({ text, position }: { text: string; position: [number, number, number] }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  const { camera } = useThree() as { camera: THREE.PerspectiveCamera; gl: THREE.WebGLRenderer };
  const [color, setColor] = useState<THREE.Color>(new THREE.Color(0xffffff));

  // Determine the theme and set color (dark mode or light mode)
  const updateColor = () => {
    const isDarkMode = document.documentElement.classList.contains("dark");
    setColor(new THREE.Color(isDarkMode ? 0xffffff : 0x000000));
  };

  // Observe theme changes
  useEffect(() => {
    // Create a MutationObserver to watch for changes in current tab (mode toggle)
    const observer = new MutationObserver(updateColor);
    observer.observe(document.documentElement, { attributes: true, attributeFilter: ["class"] });

    // Add event listener to update color when the window is focused (tab switched)
    const handleFocus = () => updateColor();
    window.addEventListener("focus", handleFocus);

    // Initialise on mount
    updateColor();

    return () => {
      observer.disconnect();
      window.removeEventListener("focus", handleFocus);
    };
  }, []);

  // Load the font and create the text geometry
  useEffect(() => {
    const loader = new FontLoader();
    loader.load("/fonts/roboto/Roboto Medium_Regular.json", (font: any) => {
      const textGeometry = new TextGeometry(text, {
        font: font,
        size: 0.2,
        height: 0,
      });

      if (meshRef.current) {
        meshRef.current.geometry = textGeometry;
      }
    });
  }, [text]);

  // Update the labels to always face the camera
  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.lookAt(camera.position);
    }
  });

  return (
    <mesh ref={meshRef} position={position}>
      <meshBasicMaterial color={color} />
    </mesh>
  );
};
