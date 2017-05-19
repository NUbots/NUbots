import { autorun, IReactionDisposer } from 'mobx'
import { runInAction } from 'mobx'
import { inject, observer } from 'mobx-react'
import * as React from 'react'
import { WebGLRenderer } from 'three'
import { LocalisationModel } from './model'
import { ViewMode } from './model'
import * as style from './style.css'
import { LocalisationViewModel } from './view_model'

@inject('localisationStore')
@observer
export class LocalisationView extends React.Component<any, any> {
  private canvas: HTMLCanvasElement
  private renderer: WebGLRenderer
  private stopAutorun: IReactionDisposer
  private rafId: number

  constructor(props: any, context: any) {
    super(props, context)
  }

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
    const model = this.props.localisationStore as LocalisationModel
    return (
        <div className={style.localisation}>
          <MenuBar onHawkEyeClick={this.onHawkEyeClick}/>
          <div className={style.localisation__canvasContainer}>
            <canvas className={style.localisation__canvas} ref={canvas => {
              this.canvas = canvas
            }}/>
          </div>
          <StatusBar model={model}/>
        </div>
    )
  }

  public requestPointerLock() {
    this.canvas.requestPointerLock()
  }

  private onAnimationFrame = (time: number) => {
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
    this.props.presenter.onAnimationFrame(time)
  }

  private renderScene(): void {
    const canvas = this.canvas
    const model = this.props.localisationStore

    if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
      this.renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
      runInAction(() => model.aspect = canvas.clientWidth / canvas.clientHeight)
    }

    const viewModel = LocalisationViewModel.of(model)

    this.renderer.render(viewModel.scene, viewModel.camera)

    runInAction(() => model.time.lastRenderTime = model.time.time)
  }

  private onClick = (e: MouseEvent) => {
    if (e.button === 0) {
      this.props.presenter.onLeftClick(this)
    } else if (e.button === 2) {
      this.props.presenter.onRightClick(this)
    }
  }

  private onPointerLockChange = () => {
    this.props.presenter.onPointerLockChange(document.pointerLockElement === this.canvas)
  }

  private onMouseMove = (e: MouseEvent) => {
    this.props.presenter.onMouseMove(e.movementX, e.movementY)
  }

  private onKeyDown = (e: KeyboardEvent) => {
    this.props.presenter.onKeyDown(e.keyCode, {
      shiftKey: e.shiftKey,
      ctrlKey: e.ctrlKey,
    })
  }

  private onKeyUp = (e: KeyboardEvent) => {
    this.props.presenter.onKeyUp(e.keyCode)
  }

  private onHawkEyeClick = () => {
    this.props.presenter.onHawkEyeClick()
  }

  private onWheel = (e: WheelEvent) => {
    e.preventDefault()
    this.props.presenter.onWheel(e.deltaY)
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
  const hasTarget = props.model.viewMode !== ViewMode.NO_CLIP && props.model.target
  return (
      <div className={style.localisation__status}>
        <span className={style.localisation__info}>&#160;</span>
        <span
            className={style.localisation__target}>{hasTarget && props.model.target.name || 'No Target'}</span>
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
