import { autorun, IReactionDisposer } from 'mobx'
import { runInAction } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'
import { HTMLProps } from 'react'
import { WebGLRenderer } from 'three'
import { inject } from '../../../inversify.config'
import { LocalisationController } from './controller'
import { LocalisationModel } from './model'
import { ViewMode } from './model'
import * as style from './style.css'
import { LocalisationViewModel } from './view_model'

@observer
export class LocalisationView extends React.Component<HTMLProps<JSX.Element>, void> {
  @inject(LocalisationController)
  private controller: LocalisationController

  @inject(LocalisationModel)
  private model: LocalisationModel

  private canvas: HTMLCanvasElement
  private renderer: WebGLRenderer
  private stopAutorun: IReactionDisposer
  private rafId: number

  public componentDidMount(): void {
    this.renderer = new WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
    })
    this.stopAutorun = autorun(() => this.renderScene())
    this.canvas.addEventListener('click', this.onClick, false)
    document.addEventListener('pointerlockchange', this.onPointerLockChange, false)
    document.addEventListener('mousemove', this.onMouseMove, false)
    document.addEventListener('keydown', this.onKeyDown, false)
    document.addEventListener('keyup', this.onKeyUp, false)
    document.addEventListener('wheel', this.onWheel, false)
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
  }

  public componentWillUnmount(): void {
    this.stopAutorun()
    this.canvas.removeEventListener('click', this.onClick, false)
    document.removeEventListener('pointerlockchange', this.onPointerLockChange, false)
    document.removeEventListener('mousemove', this.onMouseMove, false)
    document.removeEventListener('keydown', this.onKeyDown, false)
    document.removeEventListener('keyup', this.onKeyUp, false)
    document.removeEventListener('wheel', this.onWheel, false)
    cancelAnimationFrame(this.rafId)
  }

  public render(): JSX.Element {
    return (
        <div className={style.localisation}>
          <MenuBar onHawkEyeClick={this.onHawkEyeClick}/>
          <div className={style.localisation__canvasContainer}>
            <canvas className={style.localisation__canvas} ref={canvas => {
              this.canvas = canvas
            }}/>
          </div>
          <StatusBar model={this.model}/>
        </div>
    )
  }

  public requestPointerLock() {
    this.canvas.requestPointerLock()
  }

  private onAnimationFrame = (time: number) => {
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
    this.controller.onAnimationFrame(this.model, time)
  }

  private renderScene(): void {
    const canvas = this.canvas

    if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
      this.renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
      runInAction(() => this.model.aspect = canvas.clientWidth / canvas.clientHeight)
    }

    const viewModel = LocalisationViewModel.of(this.model)

    this.renderer.render(viewModel.scene, viewModel.camera)

    runInAction(() => this.model.time.lastRenderTime = this.model.time.time)
  }

  private onClick = (e: MouseEvent) => {
    if (e.button === 0) {
      this.controller.onLeftClick(this.model, () => this.requestPointerLock())
    } else if (e.button === 2) {
      this.controller.onRightClick(this.model)
    }
  }

  private onPointerLockChange = () => {
    this.controller.onPointerLockChange(this.model, document.pointerLockElement === this.canvas)
  }

  private onMouseMove = (e: MouseEvent) => {
    this.controller.onMouseMove(this.model, e.movementX, e.movementY)
  }

  private onKeyDown = (e: KeyboardEvent) => {
    this.controller.onKeyDown(this.model, e.keyCode, {
      shiftKey: e.shiftKey,
      ctrlKey: e.ctrlKey,
    })
  }

  private onKeyUp = (e: KeyboardEvent) => {
    this.controller.onKeyUp(this.model, e.keyCode)
  }

  private onHawkEyeClick = () => {
    this.controller.onHawkEyeClick(this.model)
  }

  private onWheel = (e: WheelEvent) => {
    e.preventDefault()
    this.controller.onWheel(this.model, e.deltaY)
  }
}

interface MenuBarProps {
  onHawkEyeClick(): void
}

const MenuBar = observer((props: MenuBarProps) => {
  return (
      <ul className={style.localisation__menu}>
        <li className={style.localisation__menuItem}>
          <button className={style.localisation__menuButton} onClick={props.onHawkEyeClick}>Hawk Eye</button>
        </li>
      </ul>
  )
})

interface StatusBarProps {
  model: LocalisationModel
}

const StatusBar = observer((props: StatusBarProps) => {
  const target = props.model.viewMode !== ViewMode.NO_CLIP && props.model.target ? props.model.target.name : 'No Target'
  return (
      <div className={style.localisation__status}>
        <span className={style.localisation__info}>&#160;</span>
        <span
            className={style.localisation__target}>{target}</span>
        <span className={style.localisation__viewMode}>{viewModeString(props.model.viewMode)}</span>
      </div>
  )
})

function viewModeString(viewMode: ViewMode) {
  switch (viewMode) {
    case ViewMode.NO_CLIP:
      return 'Free Camera'
    case ViewMode.FIRST_PERSON:
      return 'First Person'
    case ViewMode.THIRD_PERSON:
      return 'Third Person'
    default:
      throw new Error(`No string defined for view mode ${viewMode}`)
  }
}
