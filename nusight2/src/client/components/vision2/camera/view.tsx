import { Canvas } from '@react-three/fiber'
import * as THREE from 'three'
import { action } from 'mobx'
import { observer } from 'mobx-react'
import { useMemo } from 'react'
import React from 'react'

import { SwitchesMenu } from '../../switches_menu/view'
import { ImageMesh } from '../image_view/view'
import { BallsView } from './balls'
import { CompassView } from './compass'
import { DistanceView } from './distance'
import { GoalsView } from './goals'
import { HorizonView } from './horizon'

import { CameraModel } from './model'
import styles from './styles.css'
import { GreenHorizonView } from './greenhorizon'
import { VisualMeshView } from './visual_mesh'

export type CameraViewProps = {
  model: CameraModel
}

export const CameraView = observer(({ model }: { model: CameraModel }) => {
  const camera = useMemo(() => {
    const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1, 1)
    ;(camera as any).manual = true
    return camera
  }, [])
  const { drawOptions, params, image, visualmesh, greenhorizon, balls, goals } = model

  return (
    <div className={styles.camera}>
      <Canvas
        gl={{ antialias: false, alpha: false, depth: false, stencil: false }}
        orthographic={true}
        camera={camera}
        linear={true}
        flat={true}
        style={{ backgroundColor: 'black' }}
      >
        {drawOptions.drawImage && <ImageMesh image={image} />}
        {drawOptions.drawVisualmesh && visualmesh && (
          <VisualMeshView visualMesh={visualmesh} params={params} />
        )}
        {drawOptions.drawDistance && <DistanceView params={params} />}
        {drawOptions.drawCompass && <CompassView params={params} />}
        {drawOptions.drawHorizon && <HorizonView params={params} />}
        {drawOptions.drawGreenhorizon && greenhorizon && (
          <GreenHorizonView greenHorizon={greenhorizon} params={params} />
        )}
        {drawOptions.drawBalls && balls && <BallsView balls={balls} params={params} />}
        {drawOptions.drawGoals && goals && <GoalsView goals={goals} params={params} />}
      </Canvas>
      <div className={styles.menu}>
        <SwitchesMenu dropdownMenuPosition="right" options={menuDrawOptions(model)} />
      </div>
    </div>
  )
})

function menuDrawOptions({ drawOptions }: CameraModel) {
  return [
    {
      label: 'Image',
      enabled: drawOptions.drawImage,
      toggle: action(() => (drawOptions.drawImage = !drawOptions.drawImage)),
    },
    {
      label: 'Visual Mesh',
      enabled: drawOptions.drawVisualmesh,
      toggle: action(() => (drawOptions.drawVisualmesh = !drawOptions.drawVisualmesh)),
    },
    {
      label: 'Distance',
      enabled: drawOptions.drawDistance,
      toggle: action(() => (drawOptions.drawDistance = !drawOptions.drawDistance)),
    },
    {
      label: 'Compass',
      enabled: drawOptions.drawCompass,
      toggle: action(() => (drawOptions.drawCompass = !drawOptions.drawCompass)),
    },
    {
      label: 'Horizon',
      enabled: drawOptions.drawHorizon,
      toggle: action(() => (drawOptions.drawHorizon = !drawOptions.drawHorizon)),
    },
    {
      label: 'Green Horizon',
      enabled: drawOptions.drawGreenhorizon,
      toggle: action(() => (drawOptions.drawGreenhorizon = !drawOptions.drawGreenhorizon)),
    },
    {
      label: 'Balls',
      enabled: drawOptions.drawBalls,
      toggle: action(() => (drawOptions.drawBalls = !drawOptions.drawBalls)),
    },
    {
      label: 'Goals',
      enabled: drawOptions.drawGoals,
      toggle: action(() => (drawOptions.drawGoals = !drawOptions.drawGoals)),
    },
  ]
}
