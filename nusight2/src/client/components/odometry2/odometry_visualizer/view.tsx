import styles from './styles.css'
import { action } from 'mobx'
import { computed } from 'mobx'
import React from 'react'
import { Vector2 } from '../../../math/vector2'
import { Canvas } from '../../three/three'
import { Three } from '../../three/three'
import { OdometryVisualizerModel } from './model'
import { OdometryVisualizerViewModel } from './view_model'

export class OdometryVisualizer extends React.Component<{ model: OdometryVisualizerModel }> {

  render() {
    return (
      <div className={styles.visualizer}>
        <div className={styles.legend}>
          <div className={styles.item}>
            Oh hi there
          </div>
        </div>
      </div>
    )
  }

}
