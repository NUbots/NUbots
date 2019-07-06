import { reaction } from 'mobx'
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
import { Color } from 'three'
import { WebGLRenderTarget } from 'three'
import { WebGLRenderer } from 'three'
import { Scene } from 'three'
import { Camera } from 'three'

import * as styles from './styles.css'

export type Stage = { scene: Scene, camera: Camera, target?: WebGLRenderTarget }
export type Canvas = { width: number, height: number }

export class Three extends Component<{
  stage(canvas: Canvas): IComputedValue<Stage | Array<IComputedValue<Stage>>>,
  clearColor?: Color,
  onClick?({ button }: { button: number }): void
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
    this.props.clearColor && this.renderer.setClearColor(this.props.clearColor)
    const stages = this.props.stage!(this.canvas)
    // TODO (Annable): Extract this and add unit tests.
    let disposeStages: () => void = () => undefined
    disposeOnUnmount(this, reaction(
      () => stages.get(),
      stages => {
        if (stages instanceof Array) {
          disposeStages()
          // Create individual reactions for each stage, so they may react and re-render independently.
          disposeStages = compose(stages.map(stage => autorun(
            () => this.renderStage(stage.get()),
            { scheduler: requestAnimationFrame },
          )))
          disposeOnUnmount(this, disposeStages)
        } else {
          this.renderStage(stages)
        }
      },
      { fireImmediately: true, scheduler: requestAnimationFrame },
    ))
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
          onClick={this.props.onClick}
          onMouseDown={this.onMouseDown}
          onMouseMove={this.onMouseMove}
          onMouseUp={this.onMouseUp}
          onWheel={this.onWheel}
        />
      </div>
    </ReactResizeDetector>
  }

  requestPointerLock() {
    this.ref!.requestPointerLock()
  }

  isPointerLocked() {
    return document.pointerLockElement === this.ref!
  }

  private renderStage(stage: Stage) {
    if (!this.renderer) {
      throw new Error()
    }
    this.renderer.setSize(this.canvas.width, this.canvas.height, false)
    this.renderer.setRenderTarget(stage.target || null)
    this.renderer.render(stage.scene, stage.camera)
    this.renderer.setRenderTarget(null)
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

/** Take an array of functions and return a function that calls them all. */
const compose = (fns: Array<() => void>): () => void => () => {
  for (const fn of fns) {
    fn()
  }
}
