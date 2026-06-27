import React, { useMemo } from "react";

/** A uniform for a shader material */
export function ShaderUniform(props: { name: string; value: any | any[] }) {
  // If the value is an array, memoize to all its contents, otherwise memoize to the value itself
  const deps = Array.isArray(props.value) ? props.value : [props.value];

  // Memoize the value(s) to the uniform object
  const obj = useMemo(() => ({ value: props.value }), deps);

  // Attach the uniform object to the shader uniforms
  return <primitive attach={"uniforms-" + props.name} object={obj} />;
}
