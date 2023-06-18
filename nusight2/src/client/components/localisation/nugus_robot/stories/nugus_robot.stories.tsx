import { observer } from 'mobx-react'
import React from 'react'
import { Meta, StoryObj } from '@storybook/react'
import { reaction } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { now } from 'mobx-utils'

import { Vector3 } from '../../../../../shared/math/vector3'
import { RobotModel } from '../../../robot/model'
import { fullscreen } from '../../../storybook/fullscreen'
import { LocalisationRobotModel } from '../../robot_model'
import { NUgusView } from '../view'

import { ModelVisualiser } from './model_visualizer'

interface StoryProps {
}

const meta: Meta<StoryProps> = {
  title: 'components/localisation/NUgus Robot',
  parameters: { layout: 'fullscreen' },
  decorators: [ fullscreen ],
}

export default meta

export const Static: StoryObj<StoryProps> = {
  render: () => <NUgusVisualizer/>,
}

export const Animated: StoryObj<StoryProps> = {
  render: () => <NUgusVisualizer animate/>,
}

@observer
class NUgusVisualizer extends React.Component<{ animate?: boolean }> {
  private model = this.createModel();

  componentDidMount() {
    this.props.animate && disposeOnUnmount(
      this,
      reaction(
        () => (2 * Math.PI * now('frame')) / 1000,
        (t) => this.simulateWalk(this.model, t),
        { fireImmediately: true },
      ),
    )
  }

  render() {
    return <ModelVisualiser cameraPosition={new Vector3(0.5, 0.6, 0.5)}>
      <NUgusView model={this.model}/>
    </ModelVisualiser>
  }

  private createModel() {
    const robotModel = RobotModel.of({
      id: 'Darwin #1',
      connected: true,
      enabled: true,
      name: 'Darwin #1',
      address: '127.0.0.1',
      port: 1234,
    })
    return LocalisationRobotModel.of(robotModel)
  }

  simulateWalk(model: LocalisationRobotModel, t: number) {
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
}
