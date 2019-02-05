import { IComputedValue } from 'mobx'
import { observable } from 'mobx'
import { action } from 'mobx'
import { autorun } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { WheelEvent } from 'react'
import { MouseEvent } from 'react'
import * as React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'
import { PerspectiveCamera } from 'three'
import { WebGLRenderer } from 'three'
import { Scene } from 'three'
import { Camera } from 'three'

import * as styles from './styles.css'

export type Stage = { scene: Scene, camera: Camera }
export type Canvas = { width: number, height: number }

export class Three extends Component<{
  stage(canvas: Canvas): IComputedValue<Stage>,
  onMouseDown?(x: number, y: number): void
  onMouseMove?(x: number, y: number): void
  onMouseUp?(x: number, y: number): void
  onWheel?(deltaY: number, preventDefault: () => void): void
}> {
  @observable private canvas: Canvas = { width: 0, height: 0 }
  private ref: HTMLCanvasElement | null = null
  private renderer?: WebGLRenderer

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
        <canvas
          ref={this.setRef}
          className={styles.canvas}
          onMouseDown={this.onMouseDown}
          onMouseMove={this.onMouseMove}
          onMouseUp={this.onMouseUp}
          onWheel={this.onWheel}
        />
      </div>
    </ReactResizeDetector>
  }

  private renderStage(stage: Stage) {
    this.renderer!.setSize(this.canvas.width, this.canvas.height, false)
    this.renderer!.render(stage.scene, stage.camera)
  }

  @action.bound
  private onResize(width: number, height: number) {
    this.canvas.width = width
    this.canvas.height = height
  }

  private readonly setRef = (ref: HTMLCanvasElement | null) => {
    this.ref = ref
  }

  private onMouseDown = (e: MouseEvent<HTMLCanvasElement>) => {
    this.props.onMouseDown && this.props.onMouseDown(e.nativeEvent.layerX, e.nativeEvent.layerY)
  }

  private onMouseMove = (e: MouseEvent<HTMLCanvasElement>) => {
    this.props.onMouseMove && this.props.onMouseMove(e.nativeEvent.layerX, e.nativeEvent.layerY)
  }

  private onMouseUp = (e: MouseEvent<HTMLCanvasElement>) => {
    this.props.onMouseUp && this.props.onMouseUp(e.nativeEvent.layerX, e.nativeEvent.layerY)
  }

  private onWheel = (e: WheelEvent<HTMLCanvasElement>) => {
    this.props.onWheel && this.props.onWheel(e.deltaY, () => e.preventDefault())
  }
}
