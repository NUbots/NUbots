import { Canvas, useThree } from "@react-three/fiber";
import React from "react";
import * as THREE from "three";

export const ThreeFiber = ({ children }: { children: React.ReactNode }) => (
  <Canvas frameloop="demand" linear flat gl={{ antialias: true }} style={{ background: "black" }}>
    {children}
  </Canvas>
);

export const PerspectiveCamera = (
  props: JSX.IntrinsicElements["perspectiveCamera"] & {
    /** Making it manual will stop responsiveness, and you have to calculate aspect ratio yourself. */
    manual?: boolean;
  },
) => {
  const ref = React.useRef<THREE.PerspectiveCamera>(null);
  const three = useThree();
  React.useEffect(() => {
    const camera = ref.current;
    if (camera) {
      three.set({ camera });
      three.setSize(three.gl.domElement.clientWidth, three.gl.domElement.clientHeight);
    }
  }, []);
  return <perspectiveCamera ref={ref} {...props} />;
};
