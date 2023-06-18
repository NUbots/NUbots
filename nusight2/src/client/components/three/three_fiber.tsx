import React from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { CanvasProps } from "@react-three/fiber/dist/declarations/src/web/Canvas";
import * as THREE from "three";

import { Vector3 } from "../../../shared/math/vector3";

type Props = { children: React.ReactNode } & CanvasProps;
export const ThreeFiber = React.forwardRef<HTMLCanvasElement, Props>(({ children, ...props }: Props, ref) => (
  <Canvas ref={ref} frameloop="demand" linear flat gl={{ antialias: true }} style={{ background: "black" }} {...props}>
    {children}
  </Canvas>
));

export const PerspectiveCamera = ({
  lookAt,
  ...props
}: Omit<JSX.IntrinsicElements["perspectiveCamera"], "lookAt"> & {
  /** Making it manual will stop responsiveness, and you have to calculate aspect ratio yourself. */
  manual?: boolean;
  lookAt?: Vector3;
}) => {
  const ref = React.useRef<THREE.PerspectiveCamera>(null);
  const three = useThree();
  React.useEffect(() => {
    const camera = ref.current;
    if (camera) {
      three.set({ camera });
      three.setSize(three.gl.domElement.clientWidth, three.gl.domElement.clientHeight);
      lookAt && camera.lookAt(lookAt.toThree());
    }
  }, [lookAt]);
  return <perspectiveCamera ref={ref} {...props} />;
};
