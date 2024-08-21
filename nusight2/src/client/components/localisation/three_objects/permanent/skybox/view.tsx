import React from "react";
import { computed } from "mobx";
import * as THREE from "three";

import { Vector3 } from "../../../../../../shared/math/vector3";

import { SkyboxModel } from "./model";
import skyboxFrag from "./skybox.frag";
import skyboxVert from "./skybox.vert";

export class SkyboxView extends React.Component<{
  model: SkyboxModel;
}> {
  private get model() {
    return this.props.model;
  }

  render() {
    // reference: http://threejs.org/examples/#webgl_shaders_sky
    return (
      <object3D>
        <mesh name="skyboxSky">
          <sphereBufferGeometry args={[40, 32, 15]} />
          <shaderMaterial
            vertexShader={skyboxVert}
            fragmentShader={skyboxFrag}
            uniforms={{
              luminance: { value: this.model.luminance },
              turbidity: { value: this.model.turbidity },
              rayleigh: { value: this.model.rayleigh },
              mieCoefficient: { value: this.model.mieCoefficient },
              mieDirectionalG: { value: this.model.mieDirectionalG },
              sunPosition: { value: this.sunPosition },
            }}
            side={THREE.BackSide}
          />
        </mesh>
        <mesh name="skyboxSun" position={this.sunPosition.toArray()} visible={this.model.showSun}>
          <sphereBufferGeometry args={[40, 16, 8]} />
          <meshBasicMaterial color="white" />
        </mesh>
      </object3D>
    );
  }

  @computed
  private get sunPosition(): Vector3 {
    const distance = 40;
    const theta = Math.PI * (this.model.inclination - 0.5);
    const phi = 2 * Math.PI * (this.model.azimuth - 0.5);

    return new Vector3(
      distance * Math.cos(phi),
      distance * Math.sin(phi) * Math.sin(theta),
      distance * Math.sin(phi) * Math.cos(theta),
    );
  }
}
