import { PropsWithChildren } from "react";
import React from "react";
import { ComponentType } from "react";
import { reaction } from "mobx";
import { computed } from "mobx";
import { disposeOnUnmount } from "mobx-react";
import { observer } from "mobx-react";
import { now } from "mobx-utils";

import { Canvas } from "../three/three";
import { Three } from "../three/three";

import { LocalisationController } from "./controller";
import { LocalisationModel } from "./model";
import { ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import style from "./style.module.css";
import { LocalisationViewModel } from "./view_model";

type LocalisationViewProps = {
  controller: LocalisationController;
  Menu: ComponentType<{}>;
  model: LocalisationModel;
  network: LocalisationNetwork;
};

@observer
export class LocalisationView extends React.Component<LocalisationViewProps> {
  private readonly canvas = React.createRef<Three>();

  componentDidMount(): void {
    document.addEventListener("pointerlockchange", this.onPointerLockChange, false);
    document.addEventListener("mousemove", this.onMouseMove, false);
    document.addEventListener("keydown", this.onKeyDown, false);
    document.addEventListener("keyup", this.onKeyUp, false);
    document.addEventListener("wheel", this.onWheel, false);
    disposeOnUnmount(
      this,
      reaction(() => now("frame"), this.onAnimationFrame),
    );
  }

  componentWillUnmount(): void {
    document.removeEventListener("pointerlockchange", this.onPointerLockChange, false);
    document.removeEventListener("mousemove", this.onMouseMove, false);
    document.removeEventListener("keydown", this.onKeyDown, false);
    document.removeEventListener("keyup", this.onKeyUp, false);
    document.removeEventListener("wheel", this.onWheel, false);
    this.props.network.destroy();
  }

  render(): JSX.Element {
    return (
      <div className={style.localisation}>
        <LocalisationMenuBar Menu={this.props.Menu} onHawkEyeClick={this.onHawkEyeClick} />
        <div className={style.localisation__canvas}>
          <Three ref={this.canvas} onClick={this.onClick} stage={this.stage} />
        </div>
        <StatusBar model={this.props.model} />
      </div>
    );
  }

  private stage = (canvas: Canvas) => computed(() => [LocalisationViewModel.of(canvas, this.props.model).stage]);

  requestPointerLock() {
    this.canvas.current!.requestPointerLock();
  }

  private onAnimationFrame = (time: number) => {
    this.props.controller.onAnimationFrame(this.props.model, time);
  };

  private onClick = (e: { button: number }) => {
    if (e.button === 0) {
      this.props.controller.onLeftClick(this.props.model, () => this.requestPointerLock());
    } else if (e.button === 2) {
      this.props.controller.onRightClick(this.props.model);
    }
  };

  private onPointerLockChange = () => {
    this.props.controller.onPointerLockChange(this.props.model, this.canvas.current!.isPointerLocked());
  };

  private onMouseMove = (e: MouseEvent) => {
    this.props.controller.onMouseMove(this.props.model, e.movementX, e.movementY);
  };

  private onKeyDown = (e: KeyboardEvent) => {
    this.props.controller.onKeyDown(this.props.model, e.keyCode, {
      shiftKey: e.shiftKey,
      ctrlKey: e.ctrlKey,
    });
  };

  private onKeyUp = (e: KeyboardEvent) => {
    this.props.controller.onKeyUp(this.props.model, e.keyCode);
  };

  private onHawkEyeClick = () => {
    this.props.controller.onHawkEyeClick(this.props.model);
  };

  private onWheel = (e: WheelEvent) => {
    e.preventDefault();
    this.props.controller.onWheel(this.props.model, e.deltaY);
  };
}

interface LocalisationMenuBarProps {
  Menu: ComponentType<PropsWithChildren>;

  onHawkEyeClick(): void;
}

const LocalisationMenuBar = observer((props: LocalisationMenuBarProps) => {
  const { Menu } = props;
  return (
    <Menu>
      <ul className={style.localisation__menu}>
        <li className={style.localisation__menuItem}>
          <button className={style.localisation__menuButton} onClick={props.onHawkEyeClick}>
            Hawk Eye
          </button>
        </li>
      </ul>
    </Menu>
  );
});

interface StatusBarProps {
  model: LocalisationModel;
}

const StatusBar = observer((props: StatusBarProps) => {
  const target =
    props.model.viewMode !== ViewMode.FreeCamera && props.model.target ? props.model.target.name : "No Target";
  return (
    <div className={style.localisation__status}>
      <span className={style.localisation__info}>&#160;</span>
      <span className={style.localisation__target}>{target}</span>
      <span className={style.localisation__viewMode}>{viewModeString(props.model.viewMode)}</span>
    </div>
  );
});

function viewModeString(viewMode: ViewMode) {
  switch (viewMode) {
    case ViewMode.FreeCamera:
      return "Free Camera";
    case ViewMode.FirstPerson:
      return "First Person";
    case ViewMode.ThirdPerson:
      return "Third Person";
    default:
      throw new Error(`No string defined for view mode ${viewMode}`);
  }
}
