import { useEffect, useRef, useState } from "react";
import { useThree } from "@react-three/fiber";
import * as THREE from "three";

export const CameraControls = () => {
  const { camera, gl } = useThree() as { camera: THREE.PerspectiveCamera; gl: THREE.WebGLRenderer };
  const [isDragging, setIsDragging] = useState(false);
  const [isPanning, setIsPanning] = useState(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(camera.fov);

  const spherical = useRef(new THREE.Spherical(20, Math.PI / 2.5, Math.PI / 4)).current;
  const lookAtOffset = useRef(new THREE.Vector3(0, -2, 0));

  // Update camera position and view
  useEffect(() => {
    camera.position.setFromSpherical(spherical).add(lookAtOffset.current);
    camera.lookAt(lookAtOffset.current);
    camera.updateProjectionMatrix();
  }, [camera, spherical]);

  // Handle mouse interactions
  useEffect(() => {
    // Determine if the mouse is panning or rotating
    const handleMouseDown = (event: MouseEvent) => {
      if (event.ctrlKey) {
        setIsPanning(true); // Ctrl + left drag -> Panning
      } else {
        setIsDragging(true); // Left drag -> Rotation
      }
      dragStart.current = { x: event.clientX, y: event.clientY };
    };

    // Release mouse button
    const handleMouseUp = () => {
      setIsDragging(false);
      setIsPanning(false);
    };

    // Handle mouse movement for panning or rotating
    const handleMouseMove = (event: MouseEvent) => {
      if (!isDragging && !isPanning) return;

      const deltaX = event.clientX - dragStart.current.x;
      const deltaY = event.clientY - dragStart.current.y;
      dragStart.current = { x: event.clientX, y: event.clientY };

      if (isPanning) {
        // Calculate panning movement based on camera direction
        const panSpeed = 0.02;
        const right = new THREE.Vector3();
        camera.getWorldDirection(right);
        right
          .cross(camera.up)
          .normalize()
          .multiplyScalar(-deltaX * panSpeed);

        const up = new THREE.Vector3(0, 1, 0).multiplyScalar(deltaY * panSpeed);
        lookAtOffset.current.add(right).add(up);
      } else {
        // Rotate the camera
        spherical.theta -= deltaX * 0.005;
        spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi - deltaY * 0.005));
      }

      // Update camera position and view
      camera.position.setFromSpherical(spherical).add(lookAtOffset.current);
      camera.lookAt(lookAtOffset.current);
    };

    // Handle mouse wheel for zooming
    const handleWheel = (event: WheelEvent) => {
      event.preventDefault();
      setZoom((prevZoom: number) => {
        const newZoom = Math.min(Math.max(prevZoom + event.deltaY * 0.05, 10), 80);
        return newZoom;
      });
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
  }, [camera, gl, isDragging, isPanning, spherical]);

  // Update zoom
  useEffect(() => {
    camera.fov = zoom;
    camera.updateProjectionMatrix();
  }, [zoom, camera]);

  return null;
};
