import React from "react";
import { PropsWithChildren } from "react";
import { ComponentType } from "react";
import { reaction } from "mobx";
import { observer } from "mobx-react";
import { disposeOnUnmount } from "mobx-react";
import { now } from "mobx-utils";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { Vector3 } from "../../../shared/math/vector3";
import { Icon } from "../icon/view";
import { PerspectiveCamera } from "../three/three_fiber";
import { ThreeFiber } from "../three/three_fiber";

import { LocalisationController } from "./controller";
import { FieldView } from "./field/view";
import { GridView } from "./grid/view";
import { LocalisationModel } from "./model";
import { ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { LocalisationRobotModel } from "./robot_model";
import { SkyboxView } from "./skybox/view";
import style from "./style.module.css";

type LocalisationViewProps = {
  controller: LocalisationController;
  Menu: ComponentType<{}>;
  model: LocalisationModel;
  network: LocalisationNetwork;
};

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

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
        <LocalisationMenuBar
          model={this.props.model}
          Menu={this.props.Menu}
          onHawkEyeClick={this.onHawkEyeClick}
          toggleGridVisibility={this.toggleGridVisibility}
          toggleFieldVisibility={this.toggleFieldVisibility}
          toggleRobotVisibility={this.toggleRobotVisibility}
          toggleBallVisibility={this.toggleBallVisibility}
          toggleFieldLinePointsVisibility={this.toggleFieldLinePointsVisibility}
          toggleTrajectoryVisibility={this.toggleTrajectoryVisibility}
        ></LocalisationMenuBar>
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

  private toggleGridVisibility = () => {
    this.props.controller.toggleGridVisibility(this.props.model);
  };

  private toggleFieldVisibility = () => {
    this.props.controller.toggleFieldVisibility(this.props.model);
  };

  private toggleRobotVisibility = () => {
    this.props.controller.toggleRobotVisibility(this.props.model);
  };

  private toggleBallVisibility = () => {
    this.props.controller.toggleBallVisibility(this.props.model);
  };

  private toggleFieldLinePointsVisibility = () => {
    this.props.controller.toggleFieldLinePointsVisibility(this.props.model);
  };

  private toggleTrajectoryVisibility = () => {
    this.props.controller.toggleTrajectoryVisibility(this.props.model);
  };
}

interface LocalisationMenuBarProps {
  Menu: ComponentType<PropsWithChildren>;

  model: LocalisationModel;

  onHawkEyeClick(): void;
  toggleGridVisibility(): void;
  toggleFieldVisibility(): void;
  toggleRobotVisibility(): void;
  toggleBallVisibility(): void;
  toggleFieldLinePointsVisibility(): void;
  toggleTrajectoryVisibility(): void;
}

const MenuItem = (props: { label: string; onClick(): void; isVisible: boolean }) => {
  return (
    <li className={style.localisation__menuItem}>
      <button className={style.localisation__menuButton} onClick={props.onClick}>
        <div className="flex items-center justify-center">
          <div className="flex items-center rounded">
            <span className="mx-2">{props.label}</span>
            <Icon size={24}>{props.isVisible ? "check_box" : "check_box_outline_blank"}</Icon>
          </div>
        </div>
      </button>
    </li>
  );
};

const LocalisationMenuBar = observer((props: LocalisationMenuBarProps) => {
  const { Menu, model } = props;
  return (
    <Menu>
      <ul className={style.localisation__menu}>
        <li className={style.localisation__menuItem}>
          <button className={style.localisation__menuButton} onClick={props.onHawkEyeClick}>
            Hawk Eye
          </button>
        </li>
        <MenuItem label="Grid" isVisible={model.gridVisible} onClick={props.toggleGridVisibility} />
        <MenuItem label="Field" isVisible={model.fieldVisible} onClick={props.toggleFieldVisibility} />
        <MenuItem label="Robots" isVisible={model.robotVisible} onClick={props.toggleRobotVisibility} />
        <MenuItem label="Balls" isVisible={model.ballVisible} onClick={props.toggleBallVisibility} />
        <MenuItem
          label="Field Line Points"
          isVisible={model.fieldLinePointsVisible}
          onClick={props.toggleFieldLinePointsVisibility}
        />
        <MenuItem label="Trajectories" isVisible={model.trajectoryVisible} onClick={props.toggleTrajectoryVisibility} />
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
      <SkyboxView model={model.skybox} />
      <hemisphereLight args={["#fff", "#fff", 0.6]} />
      {model.fieldVisible && <FieldView model={model.field} />}
      {model.gridVisible && <GridView />}
      {model.robotVisible &&
        model.robots.map((robotModel) => {
          return robotModel.visible && <Robot key={robotModel.id} model={robotModel} />;
        })}
      {model.fieldLinePointsVisible && <FieldLinePoints model={model} />}
      {model.ballVisible && <Balls model={model} />}

      {model.trajectoryVisible &&
        model.robots.map((robotModel) => {
          const swingFootTrajectory = robotModel.rSFf.map((d) => new THREE.Vector3(d.x, d.y, d.z));
          if (swingFootTrajectory.length > 0 && robotModel.visible) {
            const color = robotModel.walkPhase == 1 ? "#9A99D7" : "#8AC48E";
            return (
              <Trajectory trajectory={swingFootTrajectory} key={robotModel.id} lineColor={color} tubeRadius={0.005} />
            );
          }
        })}
      {model.robots.map((robotModel) => {
        const torsoTrajectory = robotModel.rTFf.map((d) => new THREE.Vector3(d.x, d.y, d.z));
        if (torsoTrajectory.length > 0 && robotModel.visible) {
          return <Trajectory trajectory={torsoTrajectory} key={robotModel.id} lineColor="grey" tubeRadius={0.005} />;
        }
      })}

      {model.trajectoryVisible &&
        model.robots.map((robotModel) => {
          if (robotModel.visible && robotModel.swingFootTrajectoryHistory.trajectories.length > 0) {
            return robotModel.swingFootTrajectoryHistory.trajectories.map((traj, index) => {
              const swingFootTrajectory = traj.trajectory.map((d) => new THREE.Vector3(d.x, d.y, d.z));
              if (swingFootTrajectory.length > 0) {
                const color = traj.walkPhase == 1 ? "#9A99D7" : "#8AC48E";
                return (
                  <Trajectory
                    trajectory={swingFootTrajectory}
                    key={`${robotModel.id}-trajectory-${index}`}
                    lineColor={color}
                    tubeRadius={0.005}
                  />
                );
              }
            });
          }
        })}
      {model.trajectoryVisible &&
        model.robots.map((robotModel) => {
          if (robotModel.visible && robotModel.torsoTrajectoryHistory.trajectories.length > 0) {
            return robotModel.torsoTrajectoryHistory.trajectories.map((trajectory, index) => {
              const torsoTrajectory = trajectory.map((d) => new THREE.Vector3(d.x, d.y, d.z));
              if (torsoTrajectory.length > 0) {
                return (
                  <Trajectory
                    trajectory={torsoTrajectory}
                    key={`${robotModel.id}-trajectory-${index}`}
                    lineColor="grey"
                    tubeRadius={0.005}
                  />
                );
              }
            });
          }
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

const Trajectory = ({
  trajectory,
  lineColor,
  tubeRadius,
}: {
  trajectory: THREE.Vector3[];
  lineColor: any;
  tubeRadius: number;
}) => {
  // Create ref
  const trajectoryRef = React.useRef<THREE.Mesh>(null);

  // React effect
  React.useEffect(() => {
    if (trajectoryRef.current) {
      // Generate spline
      const curve = new THREE.CatmullRomCurve3(trajectory);

      // Create the tube geometry along the spline
      const geometry = new THREE.TubeGeometry(curve, 50, tubeRadius, 8, false);
      const material = new THREE.MeshStandardMaterial({ color: lineColor });
      const newTrajectory = new THREE.Mesh(geometry, material);

      // Clear the old trajectory and add the new one
      trajectoryRef.current?.clear();
      trajectoryRef.current?.add(newTrajectory);
    }
  }, [trajectory, tubeRadius]);

  return <object3D ref={trajectoryRef} />;
};

const Robot = ({ model }: { model: LocalisationRobotModel }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(nugusUrdfPath, (robot: URDFRobot) => {
      if (robotRef.current) {
        robotRef.current.add(robot);
      }
    });
  }, [nugusUrdfPath]);

  const position = model.Hft.decompose().translation;
  const rotation = model.Hft.decompose().rotation;
  const motors = model.motors;

  React.useEffect(() => {
    // Update robot's pose
    if (robotRef.current) {
      robotRef.current.position.copy(new THREE.Vector3(position.x, position.y, position.z));
      robotRef.current.quaternion.copy(new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
      const joints = (robotRef.current?.children[0] as any)?.joints;
      // Update robot's joints
      if (joints) {
        joints?.head_pitch.setJointValue(motors.headTilt.angle);
        joints?.left_ankle_pitch.setJointValue(motors.leftAnklePitch.angle);
        joints?.left_ankle_roll.setJointValue(motors.leftAnkleRoll.angle);
        joints?.left_elbow_pitch.setJointValue(motors.leftElbow.angle);
        joints?.left_hip_pitch.setJointValue(motors.leftHipPitch.angle);
        joints?.left_hip_roll.setJointValue(motors.leftHipRoll.angle);
        joints?.left_hip_yaw.setJointValue(motors.leftHipYaw.angle);
        joints?.left_knee_pitch.setJointValue(motors.leftKnee.angle);
        joints?.left_shoulder_pitch.setJointValue(motors.leftShoulderPitch.angle);
        joints?.left_shoulder_roll.setJointValue(motors.leftShoulderRoll.angle);
        joints?.neck_yaw.setJointValue(motors.headPan.angle);
        joints?.right_ankle_pitch.setJointValue(motors.rightAnklePitch.angle);
        joints?.right_ankle_roll.setJointValue(motors.rightAnkleRoll.angle);
        joints?.right_elbow_pitch.setJointValue(motors.rightElbow.angle);
        joints?.right_hip_pitch.setJointValue(motors.rightHipPitch.angle);
        joints?.right_hip_roll.setJointValue(motors.rightHipRoll.angle);
        joints?.right_hip_yaw.setJointValue(motors.rightHipYaw.angle);
        joints?.right_knee_pitch.setJointValue(motors.rightKnee.angle);
        joints?.right_shoulder_pitch.setJointValue(motors.rightShoulderPitch.angle);
        joints?.right_shoulder_roll.setJointValue(motors.rightShoulderRoll.angle);
      }
    }
  }, [position, rotation, motors]);

  // Update the material of the robot
  const material = new THREE.MeshStandardMaterial({
    color: "#666666",
    roughness: 0.5,
    metalness: 0.2,
  });
  if (robotRef.current) {
    robotRef.current.traverse((child) => {
      if (child.type === "URDFVisual" && child.children.length > 0) {
        const mesh = child.children[0] as THREE.Mesh;
        mesh.material = material;
      }
    });
  }

  return <object3D ref={robotRef} />;
};
