import React from "react";
import { RawShaderMaterialProps } from "@react-three/fiber";
import { Canvas, CanvasProps, useThree } from "@react-three/fiber";
import classNames from "classnames";
import * as THREE from "three";

import { Vector3 } from "../../../shared/math/vector3";

import style from "./style.module.css";

type Props = { children: React.ReactNode; objectFit?: ObjectFit } & CanvasProps;

// Based on the object-fit CSS property: https://developer.mozilla.org/en-US/docs/Web/CSS/object-fit
export type ObjectFit =
  // Stretch content to fill entire container.
  | { type: "fill" }
  // Either cover the container with content, or contain the content in the container, while maintaining aspect ratio.
  | { type: "contain" | "cover"; aspect: number };

export const ThreeFiber = React.forwardRef<HTMLCanvasElement, Props>(
  ({ children, objectFit = { type: "fill" }, ...props }: Props, ref) => (
    <div className={style.canvas}>
      <Canvas
        ref={ref}
        frameloop="demand"
        linear
        flat
        gl={{ antialias: true }}
        className={classNames({
          [style.cover]: objectFit.type === "cover",
          [style.contain]: objectFit.type === "contain",
          [style.fill]: objectFit.type === "fill",
        })}
        style={{
          background: "black",
          // Use 'none' instead of fill to avoid stretching the visuals during a resize.
          objectFit: objectFit.type === "fill" ? "none" : objectFit.type,
        }}
        {...props}
      >
        {children}
      </Canvas>
    </div>
  ),
);

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
      if (!uniforms[key]) {
        uniforms[key] = { value: props.uniforms[key].value };
      } else {
        uniforms[key].value = props.uniforms[key].value;
      }
    }
  }
  return <rawShaderMaterial {...props} uniforms={uniforms} />;
};
