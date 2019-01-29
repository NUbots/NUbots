import { IComputedValue } from 'mobx'
import { action } from 'mobx'
import { autorun } from 'mobx'
import { observable } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'
import { Object3D } from 'three'
import { WebGLRenderer } from 'three'
import { Scene } from 'three'
import { Camera } from 'three'

import * as styles from './styles.css'

export type Stage = { scene: Scene, camera: Camera }
export type Canvas = { width: number, height: number }

export class Three extends Component<{ stage(canvas: Canvas): IComputedValue<Stage> }> {
  @observable private canvas: Canvas = { width: 0, height: 0 }
  private ref: HTMLCanvasElement | null = null
  private renderer?: WebGLRenderer
  /**
   * Internally, three.js keeps a mapping between scene/camera pairs and various resources [1].
   * This means we need to pass the same scene and camera object to the renderer every frame.
   *
   * [1]: https://goo.gl/81PqNi
   */
  private stage = new StageCache()

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
    this.stage.copy(stage)
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

export class StageCache {
  private readonly camerasPerType = new Map<string, Camera>()
  latestCamera?: Camera
  readonly scene = new Scene()

  get camera(): Camera {
    if (!this.latestCamera) {
      throw new Error('Precondition: No camera available')
    }
    return this.latestCamera
  }

  copy(stage: Stage): Stage {
    this.copyToLocalScene(stage.scene)
    this.copyToLocalCamera(stage.camera)
    return { camera: this.camera, scene: this.scene }
  }

  private copyToLocalScene(scene: Scene) {
    this.scene.copy(scene)
    this.scene.children.length && this.scene.remove(...this.scene.children)
    for (const child of scene.children) {
      // Hack: Remove the parent so that three.js doesn't remove the children from the original scene object.
      // https://github.com/mrdoob/three.js/blob/bf062b1c8e10d0516f8bb3f59e537e59994589c3/src/core/Object3D.js#L382-L386
      (child.parent as Object3D | null) = null
      this.scene.add(child)
    }
  }

  private copyToLocalCamera(camera: Camera) {
    // Store a camera per type, to handle if we switch camera type.
    let cachedCamera = this.camerasPerType.get(camera.type)
    if (!cachedCamera) {
      cachedCamera = camera.clone()
      this.camerasPerType.set(camera.type, cachedCamera)
    }
    cachedCamera.copy(camera)
    if (cachedCamera !== this.latestCamera) {
      this.latestCamera = cachedCamera
    }
  }
}
