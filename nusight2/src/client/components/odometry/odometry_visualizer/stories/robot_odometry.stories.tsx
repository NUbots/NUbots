import { storiesOf } from '@storybook/react'
import { reaction } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { now } from 'mobx-utils'
import React from 'react'
import * as THREE from 'three'
import { Matrix4 } from '../../../../math/matrix4'
import { Vector3 } from '../../../../math/vector3'
import { fullscreen } from '../../../storybook/fullscreen'
import { OdometryVisualizerModel } from '../model'
import { OdometryVisualizer } from '../view'

storiesOf('components.odometry.odometry_visualizer', module)
  .addDecorator(fullscreen)
  .add('Renders statically', () => <OdometryVisualizerHarness />)
  .add('Renders animated', () => <OdometryVisualizerHarness animate />)

class OdometryVisualizerHarness extends React.Component<{ animate?: boolean }> {
  private model = OdometryVisualizerModel.of({
    Hwt: Matrix4.fromThree(new THREE.Matrix4().makeTranslation(0, 0, 1)),
    accelerometer: new Vector3(0, 0, -9.8),
  })

  componentDidMount() {
    this.props.animate &&
      disposeOnUnmount(
        this,
        reaction(
          () => now('frame') / 1000,
          t => {
            this.model.Hwt = Matrix4.fromThree(
              new THREE.Matrix4()
                .makeRotationFromEuler(new THREE.Euler(Math.cos(t) / 5, 0, t))
                .setPosition(0, 0, 1),
            )
          },
        ),
      )
  }

  render() {
    return <OdometryVisualizer model={this.model} />
  }
}
