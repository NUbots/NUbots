import { computed } from 'mobx'
import '@react-three/fiber'
import { observer } from 'mobx-react'
import React from 'react'
import { Color } from 'three'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { Vector3 } from '../../../shared/math/vector3'
import { meshBasicMaterial } from '../three/builders'
import { circleBufferGeometry } from '../three/builders'
import { PerspectiveCamera } from '../three/three_fiber'

import { FieldView } from './field/view'
import { LocalisationModel } from './model'
import { NUgusViewModel } from './nugus_robot/view_model'
import { SkyboxViewModel } from './skybox/view_model'

export const LocalisationViewModel = observer(({ model }: { model: LocalisationModel }) => {
  return <object3D>
    <PerspectiveCamera
      args={[75, 1, 0.01, 100]}
      position={model.camera.position.toArray()}
      rotation={[Math.PI / 2 + model.camera.pitch, 0, -Math.PI / 2 + model.camera.yaw, 'ZXY']}
      up={[0, 0, 1]}
    >
      <pointLight color="white"/>
    </PerspectiveCamera>
    <FieldView model={model.field}/>
    <SkyboxViewModel model={model.skybox}/>
    <hemisphereLight args={['#fff', '#fff', 0.6]}/>
    {model.robots.map((robotModel) => {
      return robotModel.visible && <NUgusViewModel key={robotModel.id} model={robotModel}/>
    })}
    <FieldLineDots model={model}/>
  </object3D>
})

const FieldLineDots = ({ model }: { model: LocalisationModel }) => (
  <object3D>
    {model.robots.flatMap((robot) =>
      robot.fieldLinesDots.rPWw.map((d, i) => {
        return <mesh key={`${robot.id}-${i}`} position={d.add(new Vector3(0, 0, 0.005)).toArray()}>
          <circleBufferGeometry args={[0.02, 20]}/>
          <meshBasicMaterial color="blue"/>
        </mesh>
      }),
    )
    }
  </object3D>
)
