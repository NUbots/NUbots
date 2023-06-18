import React from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { CanvasProps } from "@react-three/fiber/dist/declarations/src/web/Canvas";
import * as THREE from "three";
import { mergeRefs } from 'react-merge-refs'


type Props = { children: React.ReactNode } & CanvasProps;
export const ThreeFiber = React.forwardRef<HTMLCanvasElement, Props>(({ children, ...props }: Props, ref) => (
  <Canvas ref={ref} frameloop="demand" linear flat gl={{ antialias: true }} style={{ background: "black" }} {...props}>
    {children}
  </Canvas>
));

type CameraProps = JSX.IntrinsicElements["perspectiveCamera"] & {
    /** Making it manual will stop responsiveness, and you have to calculate aspect ratio yourself. */
    manual?: boolean;
  }

export const PerspectiveCamera = React.forwardRef<THREE.PerspectiveCamera, CameraProps>((props, ref) => {
  const internalRef = React.useRef<THREE.PerspectiveCamera>(null);
  const three = useThree();
  React.useEffect(() => {
    const camera = internalRef.current;
    if (camera) {
      three.set({ camera });
      three.setSize(three.gl.domElement.clientWidth, three.gl.domElement.clientHeight);
    }
  }, []);
  return <perspectiveCamera ref={mergeRefs([ref, internalRef])} {...props} />;
});
