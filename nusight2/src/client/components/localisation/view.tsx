import React from "react";
import { PropsWithChildren } from "react";
import { ComponentType } from "react";
import { reaction } from "mobx";
import { observer } from "mobx-react";
import { disposeOnUnmount } from "mobx-react";
import { now } from "mobx-utils";

import { Vector3 } from "../../../shared/math/vector3";
import { PerspectiveCamera } from "../three/three_fiber";
import { ThreeFiber } from "../three/three_fiber";
import * as THREE from "three";

import { LocalisationController } from "./controller";
import { FieldView } from "./field/view";
import { LocalisationModel } from "./model";
import { ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { NUgusView } from "./nugus_robot/view";
import { LocalisationRobotModel } from "./robot_model";
import { SkyboxView } from "./skybox/view";
import style from "./style.module.css";

type LocalisationViewProps = {
  controller: LocalisationController;
  Menu: ComponentType<{}>;
  model: LocalisationModel;
  network: LocalisationNetwork;
};

@observer
export class LocalisationView extends React.Component<LocalisationViewProps> {
  private readonly canvas = React.createRef<HTMLCanvasElement>();

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
          <ThreeFiber ref={this.canvas} onClick={this.onClick}>
            <LocalisationViewModel model={this.props.model} />
          </ThreeFiber>
        </div>
        <StatusBar model={this.props.model} />
      </div>
    );
  }

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
    this.props.controller.onPointerLockChange(this.props.model, this.canvas.current === document.pointerLockElement);
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

export const LocalisationViewModel = observer(({ model }: { model: LocalisationModel }) => {
  return (
    <object3D>
      <PerspectiveCamera
        args={[75, 1, 0.01, 100]}
        position={model.camera.position.toArray()}
        rotation={[Math.PI / 2 + model.camera.pitch, 0, -Math.PI / 2 + model.camera.yaw, "ZXY"]}
        up={[0, 0, 1]}
      >
        <pointLight color="white" />
      </PerspectiveCamera>
      <FieldView model={model.field} />
      <SkyboxView model={model.skybox} />
      <hemisphereLight args={["#fff", "#fff", 0.6]} />
      {model.robots.map((robotModel) => {
        return robotModel.visible && <NUgusView key={robotModel.id} model={robotModel} />;
      })}
      <FieldLinePoints model={model} />
      <Balls model={model} />
      {model.robots.map((robotModel) => {
        return <SwingFootTrajectory model={robotModel} />
      })}
    </object3D>
  );
});

const FieldLinePoints = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible && (
          <object3D key={robot.id}>
            {robot.rPFf.map((d, i) => {
              return (
                <mesh key={String(i)} position={d.add(new Vector3(0, 0, 0.005)).toArray()}>
                  <circleBufferGeometry args={[0.02, 20]} />
                  <meshBasicMaterial color="blue" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);

const Balls = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible &&
        robot.rBFf && (
          <mesh position={robot.rBFf.toArray()} scale={[robot.rBFf.z, robot.rBFf.z, robot.rBFf.z]} key={robot.id}>
            <sphereBufferGeometry args={[1, 20, 20]} />
            <meshStandardMaterial color="orange" />
          </mesh>
        ),
    )}
  </>
);

const SwingFootTrajectory = ({ model }: { model: LocalisationRobotModel }) => {
  if (model.visible && model.rSFf.length != 0) {
    // Create ref
    const trajectoryRef = React.useRef<THREE.Line>(null);

    // Generate spline
    const waypoints = model.rSFf.map((d) => new THREE.Vector3(d.x, d.y, d.z));
    const curve = new THREE.CatmullRomCurve3(waypoints);

    // Get points
    const points = curve.getPoints(50);

    // React effect
    React.useEffect(() => {
      // Add the new line
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 10 });
      const trajectory = new THREE.Line(geometry, material);
      trajectoryRef.current?.clear();
      trajectoryRef.current?.add(trajectory);
    }, [points]);


    return <object3D ref={trajectoryRef} />;
  }
  return <></>;
};
