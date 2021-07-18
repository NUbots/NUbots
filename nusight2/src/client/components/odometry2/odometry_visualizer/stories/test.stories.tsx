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

storiesOf('components.odometry2.odometry_visualizer', module)
  .addDecorator(fullscreen)
  .add('Renders statically', () => <OdometryVisualizerHarness />)

class OdometryVisualizerHarness extends React.Component<{ animate?: boolean }> {
  private model = OdometryVisualizerModel.of({
    Hwt: Matrix4.fromThree(new THREE.Matrix4().makeTranslation(0, 0, 1)),
    accelerometer: new Vector3(0, 0, -9.8),
  })


  render() {
    return <OdometryVisualizer model={this.model} />
  }
}
