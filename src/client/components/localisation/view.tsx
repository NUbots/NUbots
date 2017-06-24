import { autorun } from 'mobx'
import { runInAction } from 'mobx'
import { IReactionDisposer } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'
import { HTMLProps } from 'react'
import { WebGLRenderer } from 'three'
import { MenuBar } from '../menu_bar/view'
import { LocalisationController } from './controller'
import { LocalisationModel } from './model'
import { ViewMode } from './model'
import { LocalisationNetwork } from './network'
import * as style from './style.css'
import { LocalisationViewModel } from './view_model'

interface LocalisationViewProps extends HTMLProps<JSX.Element> {
  controller: LocalisationController
  model: LocalisationModel
  network: LocalisationNetwork
}

@observer
export class LocalisationView extends React.Component<LocalisationViewProps> {
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
    this.props.network.destroy()
  }

  public render(): JSX.Element {
    return (
      <div className={style.localisation}>
        <LocalisationMenuBar onHawkEyeClick={this.onHawkEyeClick}/>
        <div className={style.localisation__canvasContainer}>
          <canvas className={style.localisation__canvas} ref={canvas => {
            if (canvas) {
              this.canvas = canvas
            }
          }}/>
        </div>
        <StatusBar model={this.props.model}/>
      </div>
    )
  }

  public requestPointerLock() {
    this.canvas.requestPointerLock()
  }

  private onAnimationFrame = (time: number) => {
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
    this.props.controller.onAnimationFrame(this.props.model, time)
  }

  private renderScene(): void {
    const canvas = this.canvas

    if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
      this.renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
      runInAction(() => this.props.model.aspect = canvas.clientWidth / canvas.clientHeight)
    }

    const viewModel = LocalisationViewModel.of(this.props.model)

    this.renderer.render(viewModel.scene, viewModel.camera)

    runInAction(() => this.props.model.time.lastRenderTime = this.props.model.time.time)
  }

  private onClick = (e: MouseEvent) => {
    if (e.button === 0) {
      this.props.controller.onLeftClick(this.props.model, () => this.requestPointerLock())
    } else if (e.button === 2) {
      this.props.controller.onRightClick(this.props.model)
    }
  }

  private onPointerLockChange = () => {
    this.props.controller.onPointerLockChange(this.props.model, document.pointerLockElement === this.canvas)
  }

  private onMouseMove = (e: MouseEvent) => {
    this.props.controller.onMouseMove(this.props.model, e.movementX, e.movementY)
  }

  private onKeyDown = (e: KeyboardEvent) => {
    this.props.controller.onKeyDown(this.props.model, e.keyCode, {
      shiftKey: e.shiftKey,
      ctrlKey: e.ctrlKey,
    })
  }

  private onKeyUp = (e: KeyboardEvent) => {
    this.props.controller.onKeyUp(this.props.model, e.keyCode)
  }

  private onHawkEyeClick = () => {
    this.props.controller.onHawkEyeClick(this.props.model)
  }

  private onWheel = (e: WheelEvent) => {
    e.preventDefault()
    this.props.controller.onWheel(this.props.model, e.deltaY)
  }
}

interface LocalisationMenuBarProps {
  onHawkEyeClick(): void
}

const LocalisationMenuBar = observer((props: LocalisationMenuBarProps) => {
  return (
    <MenuBar>
      <ul className={style.localisation__menu}>
        <li className={style.localisation__menuItem}>
          <button className={style.localisation__menuButton} onClick={props.onHawkEyeClick}>Hawk Eye</button>
        </li>
      </ul>
    </MenuBar>
  )
})

interface StatusBarProps {
  model: LocalisationModel
}

const StatusBar = observer((props: StatusBarProps) => {
  const target = props.model.viewMode !== ViewMode.FreeCamera && props.model.target
    ? props.model.target.name
    : 'No Target'
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
    case ViewMode.FreeCamera:
      return 'Free Camera'
    case ViewMode.FirstPerson:
      return 'First Person'
    case ViewMode.ThirdPerson:
      return 'Third Person'
    default:
      throw new Error(`No string defined for view mode ${viewMode}`)
  }
}
