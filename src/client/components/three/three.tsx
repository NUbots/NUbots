import { IComputedValue } from 'mobx'
import { observable } from 'mobx'
import { action } from 'mobx'
import { autorun } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'
import { PerspectiveCamera } from 'three'
import { WebGLRenderer } from 'three'
import { Scene } from 'three'
import { Camera } from 'three'

import { reconcile } from './reconcile'
import * as styles from './styles.css'

export type Stage = { scene: Scene, camera: Camera }
export type Canvas = { width: number, height: number }

export class Three extends Component<{ stage(canvas: Canvas): IComputedValue<Stage> }> {
  @observable private canvas: Canvas = { width: 0, height: 0 }
  private ref: HTMLCanvasElement | null = null
  private renderer?: WebGLRenderer
  /**
   * Internally, three.js keeps various mappings between objects and webgl resources, e.g. [1].
   * So instead we maintain our own permanent stage object and copy any updated values into it every render.
   * [1]: https://goo.gl/81PqNi
   */
  private stage: Stage = { camera: new PerspectiveCamera(), scene: new Scene() }

  componentDidMount() {
    this.renderer = new WebGLRenderer({ canvas: this.ref!, antialias: true })
    const stage = this.props.stage(this.canvas)
    disposeOnUnmount(this, autorun(() => this.renderStage(stage.get()), { scheduler: requestAnimationFrame }))
  }

  componentWillUnmount() {
    this.renderer!.dispose()
  }

  render() {
    return <ReactResizeDetector handleWidth handleHeight onResize={this.onResize}>
      <div className={styles.container}>
        <canvas ref={this.setRef} className={styles.canvas}/>
      </div>
    </ReactResizeDetector>
  }

  private renderStage(stage: Stage) {
    reconcile(stage, this.stage)
    this.renderer!.setSize(this.canvas.width, this.canvas.height, false)
    this.renderer!.render(this.stage.scene, this.stage.camera)
  }

  @action.bound
  private onResize(width: number, height: number) {
    this.canvas.width = width
    this.canvas.height = height
  }

  private readonly setRef = (ref: HTMLCanvasElement | null) => {
    this.ref = ref
  }
}
