import React from "react";
import { RawShaderMaterialProps } from "@react-three/fiber";
import { Canvas, CanvasProps, useThree } from "@react-three/fiber";
import * as THREE from "three";

import { Vector3 } from "../../../shared/math/vector3";

import style from "./style.module.css";

type Props = { children: React.ReactNode } & CanvasProps;
export const ThreeFiber = React.forwardRef<HTMLCanvasElement, Props>(({ children, ...props }: Props, ref) => (
  <div className={style.canvas}>
    <Canvas
      ref={ref}
      frameloop="demand"
      linear
      flat
      gl={{ antialias: true }}
      style={{ background: "black" }}
      {...props}
    >
      {children}
    </Canvas>
  </div>
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
  }, [lookAt, props.position, props.rotation]);
  return <perspectiveCamera ref={ref} {...props} />;
};

export const OrthographicCamera = (
  props: JSX.IntrinsicElements["orthographicCamera"] & {
    /** Making it manual will stop responsiveness, and you have to calculate aspect ratio yourself. */
    manual?: boolean;
  },
) => {
  const ref = React.useRef<THREE.OrthographicCamera>(null);
  const three = useThree();
  React.useEffect(() => {
    const camera = ref.current;
    if (camera) {
      three.set({ camera });
      three.setSize(three.gl.domElement.clientWidth, three.gl.domElement.clientHeight);
    }
  }, []);
  return <orthographicCamera ref={ref} {...props} />;
};

export const RawShaderMaterial = (props: RawShaderMaterialProps) => {
  // Maintain a constant uniforms object, as three.js does not handle it changing reference.
  const uniforms = React.useMemo<NonNullable<RawShaderMaterialProps["uniforms"]>>(() => ({}), []);
  if (props.uniforms) {
    for (const key in props.uniforms) {
      uniforms[key].value = props.uniforms[key].value;
    }
  }
  return <rawShaderMaterial {...props} uniforms={uniforms} />;
};
