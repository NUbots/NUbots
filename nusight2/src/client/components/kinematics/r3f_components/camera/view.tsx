import { useEffect, useRef, useState } from "react";
import { useThree } from "@react-three/fiber";
import * as THREE from "three";

export const CameraControls = () => {
  const { camera, gl } = useThree() as { camera: THREE.PerspectiveCamera; gl: THREE.WebGLRenderer };
  const [isDragging, setIsDragging] = useState(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(camera.fov);
  const spherical = useRef(new THREE.Spherical(20, Math.PI / 2.5, Math.PI / 4)).current;

  // Synchronise the camera position with the spherical coordinates
  useEffect(() => {
    camera.position.setFromSpherical(spherical);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
  }, [camera, spherical]);

  // Handle camera controls on mouse events
  useEffect(() => {
    const handleMouseDown = (event: MouseEvent) => {
      setIsDragging(true);
      dragStart.current = { x: event.clientX, y: event.clientY };
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    const handleMouseMove = (event: MouseEvent) => {
      if (!isDragging) return;

      const deltaX = event.clientX - dragStart.current.x;
      const deltaY = event.clientY - dragStart.current.y;
      dragStart.current = { x: event.clientX, y: event.clientY };

      spherical.theta -= deltaX * 0.005;
      spherical.phi -= deltaY * 0.005;

      spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

      camera.position.setFromSpherical(spherical);
      camera.lookAt(0, 0, 0);
    };

    const handleWheel = (event: WheelEvent) => {
      if (event.ctrlKey) {
        event.preventDefault();
        setZoom((prevZoom: number) => {
          const newZoom = Math.min(Math.max(prevZoom + event.deltaY * 0.05, 20), 100);
          return newZoom;
        });
        event.preventDefault();
      }
    };

    gl.domElement.addEventListener("mousedown", handleMouseDown);
    gl.domElement.addEventListener("mousemove", handleMouseMove);
    gl.domElement.addEventListener("mouseup", handleMouseUp);
    gl.domElement.addEventListener("wheel", handleWheel);

    return () => {
      gl.domElement.removeEventListener("mousedown", handleMouseDown);
      gl.domElement.removeEventListener("mousemove", handleMouseMove);
      gl.domElement.removeEventListener("mouseup", handleMouseUp);
      gl.domElement.removeEventListener("wheel", handleWheel);
    };
  }, [camera, gl, isDragging, spherical]);

  // Update the camera when the zoom state changes
  useEffect(() => {
    camera.fov = zoom;
    camera.updateProjectionMatrix();
  }, [zoom, camera]);

  return null;
};
