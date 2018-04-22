import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'
import { ComponentType } from 'react'

import { CameraView } from './camera/view'
import { VisionNetwork } from './network'
import * as styles from './styles.css'
import { VisionViewModel } from './view_model'

@observer
export class VisionView extends Component<{
  viewModel: VisionViewModel
  network: VisionNetwork
  Menu: ComponentType
}> {
  componentWillUnmount() {
    this.props.network.destroy()
  }

  render() {
    const { viewModel: { robots }, Menu } = this.props
    return (
      <div className={styles.vision}>
        <Menu/>
        {robots.map(({ id, name, cameras }) => (
          <div key={id}>
            <h1>{name}</h1>
            <div className={styles.cameras}>
              {cameras.map(camera => <CameraView key={camera.id} viewModel={camera}/>)}
            </div>
          </div>
        ))}
      </div>
    )
  }
}
