import React from "react";
import { observer } from "mobx-react";

import { Vector3 } from "../../../../../shared/math/vector3";
import { ThreeFiber } from "../../../three/three_fiber";
import { PerspectiveCamera } from "../../../three/three_fiber";

export const ModelVisualiser = observer(
  ({ cameraPosition, children }: { cameraPosition: Vector3; children: React.ReactNode }) => {
    return (
      <ThreeFiber>
        <PerspectiveCamera
          args={[75, 1, 0.01, 100]}
          position={cameraPosition.toArray()}
          up={[0, 0, 1]}
          lookAt={Vector3.of()}
        >
          <pointLight color="white" />
        </PerspectiveCamera>
        <axesHelper />
        <spotLight args={["#fff", 1, 20, Math.PI / 8]} position={[0, 0, 1]} />
        {children}
      </ThreeFiber>
    );
  },
);
