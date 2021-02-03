import { storiesOf } from '@storybook/react'
import { reaction } from 'mobx'
import { now } from 'mobx-utils'
import React from 'react'

import { Vector3 } from '../../../../math/vector3'
import { RobotModel } from '../../../robot/model'
import { LocalisationRobotModel } from '../model'
import { RobotViewModel } from '../view_model'

import { ModelVisualiser } from './model_visualizer'

function createModel(animate?: 'animate') {
  const robotModel = RobotModel.of({
    id: 'Darwin #1',
    connected: true,
    enabled: true,
    name: 'Darwin #1',
    address: '127.0.0.1',
    port: 1234,
  })
  const model = LocalisationRobotModel.of(robotModel)
  const viewModel = RobotViewModel.of(model)
  animate &&
    reaction(
      () => (2 * Math.PI * now('frame')) / 1000,
      t => simulateWalk(model, t),
      { fireImmediately: true },
    )
  return () => viewModel.robot
}

const cameraPosition = new Vector3(0.3, 0.4, 0.4)

storiesOf('component.localisation.darwin_robot', module)
  .add('renders statically', () => {
    return (
      <div style={{ width: 'calc(100vw - 20px)', height: 'calc(100vh - 20px)' }}>
        <ModelVisualiser model={createModel()} cameraPosition={cameraPosition} />
      </div>
    )
  })
  .add('renders animated', () => {
    return (
      <div style={{ width: 'calc(100vw - 20px)', height: 'calc(100vh - 20px)' }}>
        <ModelVisualiser model={createModel('animate')} cameraPosition={cameraPosition} />
      </div>
    )
  })

function simulateWalk(model: LocalisationRobotModel, t: number) {
  model.motors.rightShoulderPitch.angle = (3 * Math.PI) / 4 + 0.5 * Math.cos(t - Math.PI)
  model.motors.leftShoulderPitch.angle = (3 * Math.PI) / 4 + 0.5 * Math.cos(t)
  model.motors.rightShoulderRoll.angle = -Math.PI / 8
  model.motors.leftShoulderRoll.angle = Math.PI / 8
  model.motors.rightElbow.angle = (-3 * Math.PI) / 4
  model.motors.leftElbow.angle = (-3 * Math.PI) / 4
  model.motors.rightHipYaw.angle = 0
  model.motors.leftHipYaw.angle = 0
  model.motors.rightHipRoll.angle = 0
  model.motors.leftHipRoll.angle = 0
  model.motors.rightHipPitch.angle = 0.5 * (Math.cos(t) - 1)
  model.motors.leftHipPitch.angle = 0.5 * (Math.cos(t - Math.PI) - 1)
  model.motors.rightKnee.angle = 0.5 * (-Math.cos(t) + 1)
  model.motors.leftKnee.angle = 0.5 * (-Math.cos(t - Math.PI) + 1)
  model.motors.rightAnklePitch.angle = 0
  model.motors.leftAnklePitch.angle = 0
  model.motors.rightAnkleRoll.angle = 0
  model.motors.leftAnkleRoll.angle = 0
  model.motors.headPan.angle = 0.1 * Math.cos(t)
  model.motors.headTilt.angle = 0.1 * Math.cos(t / 3) + 0.4
}
