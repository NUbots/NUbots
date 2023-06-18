import { observer } from 'mobx-react'
import React from 'react'
import * as THREE from 'three'

import { Vector3 } from '../../../../../shared/math/vector3'
import { ThreeFiber } from '../../../three/three_fiber'
import { PerspectiveCamera } from '../../../three/three_fiber'

export const ModelVisualiser = observer(({ cameraPosition, children }: {
  cameraPosition: Vector3;
  children: React.ReactNode;
}) => {
  const ref = React.useRef<THREE.PerspectiveCamera>(null)
  React.useEffect(() => {
    ref.current?.lookAt(0, 0, 0)
  }, [])
  return <ThreeFiber>
    <PerspectiveCamera
      ref={ref}
      args={[75, 1, 0.01, 100]}
      position={cameraPosition.toArray()}
      up={[0, 0, 1]}
      manual={true}
    >
      <pointLight color="white"/>
    </PerspectiveCamera>
    <axesHelper/>
    <spotLight args={['#fff', 1, 20, Math.PI / 8]} position={[0, 0, 1]}/>
    {children}
  </ThreeFiber>
});
