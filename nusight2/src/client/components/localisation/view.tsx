import React, { ComponentType, PropsWithChildren } from "react";
import { reaction } from "mobx";
import { observer } from "mobx-react";
import { now } from "mobx-utils";

import { compose } from "../../../shared/base/compose";
import { Button } from "../button/button";
import { dropdownContainer } from "../dropdown_container/view";
import { Icon } from "../icon/view";
import { PerspectiveCamera, ThreeFiber } from "../three/three_fiber";

import { LocalisationController } from "./controller";
import { LocalisationModel, ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { Ball } from "./r3f_components/ball/view";
import { BoundingBox } from "./r3f_components/bounding_box/view";
import { FieldView } from "./r3f_components/field/view";
import { FieldIntersections } from "./r3f_components/field_intersections/view";
import { FieldObjects } from "./r3f_components/field_objects/view";
import { FieldPoints } from "./r3f_components/field_points/view";
import { GridView } from "./r3f_components/grid/view";
import { Nugus } from "./r3f_components/nugus/view";
import { PurposeLabel } from "./r3f_components/purpose_label/view";
import { SkyboxView } from "./r3f_components/skybox/view";
import { WalkPathGoal } from "./r3f_components/walk_path_goal/view";
import { WalkPathVisualiser } from "./r3f_components/walk_path_visualiser/view";
import { LocalisationRobotModel } from "./robot_model";

type LocalisationViewProps = {
  controller: LocalisationController;
  Menu: ComponentType<{}>;
  model: LocalisationModel;
  network: LocalisationNetwork;
};

const FieldDimensionOptions = [
  { label: "Lab", value: "lab" },
  { label: "Robocup", value: "robocup" },
];

// Apply the interfaces to the component's props
interface FieldDimensionSelectorProps {
  model: LocalisationModel;
  controller: LocalisationController;
}

@observer
export class FieldDimensionSelector extends React.Component<FieldDimensionSelectorProps> {
  private dropdownToggle = (<Button>Field Type</Button>);

  render(): JSX.Element {
    return (
      <EnhancedDropdown dropdownToggle={this.dropdownToggle}>
        <div className="bg-auto-surface-2">
          {FieldDimensionOptions.map((option) => (
            <div
              key={option.value}
              className={`flex p-2 ${
                this.props.model.field.fieldType === option.value
                  ? "hover:bg-auto-contrast-1"
                  : "hover:bg-auto-contrast-1"
              }`}
              onClick={() => this.props.controller.setFieldDimensions(option.value, this.props.model)}
            >
              <Icon size={24}>
                {this.props.model.field.fieldType === option.value ? "check_box" : "check_box_outline_blank"}
              </Icon>{" "}
              <span>{option.label}</span>
            </div>
          ))}
        </div>
      </EnhancedDropdown>
    );
  }
}

const EnhancedDropdown = dropdownContainer();

const addDocListener = <K extends keyof DocumentEventMap>(
  ...args: Parameters<typeof document.addEventListener<K>>
): (() => void) => {
  document.addEventListener(...args);
  return () => document.removeEventListener(...args);
};

@observer
export class LocalisationView extends React.Component<LocalisationViewProps> {
  private readonly canvas = React.createRef<HTMLCanvasElement>();
  private dispose?: () => void;

  componentDidMount(): void {
    this.dispose = compose([
      addDocListener("pointerlockchange", this.onPointerLockChange, false),
      addDocListener("mousemove", this.onMouseMove, false),
      addDocListener("keydown", this.onKeyDown, false),
      addDocListener("keyup", this.onKeyUp, false),
      addDocListener("wheel", this.onWheel, false),
      reaction(() => now("frame"), this.onAnimationFrame),
      () => this.props.network.destroy(),
    ]);
  }

  componentWillUnmount(): void {
    this.dispose?.();
    this.dispose = undefined;
  }

  render(): JSX.Element {
    return (
      <div className={"flex flex-grow flex-shrink flex-col relative bg-auto-surface-0"}>
        <LocalisationMenuBar
          model={this.props.model}
          Menu={this.props.Menu}
          controller={this.props.controller}
          onHawkEyeClick={this.onHawkEyeClick}
          toggleGridVisibility={this.toggleGridVisibility}
          toggleFieldVisibility={this.toggleFieldVisibility}
          toggleRobotVisibility={this.toggleRobotVisibility}
          toggleBallVisibility={this.toggleBallVisibility}
          toggleParticleVisibility={this.toggleParticleVisibility}
          toggleGoalVisibility={this.toggleGoalVisibility}
          toggleFieldLinePointsVisibility={this.toggleFieldLinePointsVisibility}
          toggleFieldIntersectionsVisibility={this.toggleFieldIntersectionsVisibility}
          toggleWalkToDebugVisibility={this.toggleWalkToDebugVisibility}
          toggleBoundedBoxVisibility={this.toggleBoundedBoxVisibility}
        ></LocalisationMenuBar>
        <div className="flex-grow relative border-t border-auto">
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

  private toggleParticleVisibility = () => {
    this.props.controller.toggleParticlesVisibility(this.props.model);
  };

  private toggleGoalVisibility = () => {
    this.props.controller.toggleGoalVisibility(this.props.model);
  };

  private toggleFieldLinePointsVisibility = () => {
    this.props.controller.toggleFieldLinePointsVisibility(this.props.model);
  };

  private toggleFieldIntersectionsVisibility = () => {
    this.props.controller.toggleFieldIntersectionsVisibility(this.props.model);
  };

  private toggleWalkToDebugVisibility = () => {
    this.props.controller.toggleWalkToDebugVisibility(this.props.model);
  };

  private toggleBoundedBoxVisibility = () => {
    this.props.controller.toggleBoundedBoxVisibility(this.props.model);
  };
}

interface LocalisationMenuBarProps {
  Menu: ComponentType<PropsWithChildren>;

  model: LocalisationModel;

  controller: LocalisationController;

  onHawkEyeClick(): void;
  toggleGridVisibility(): void;
  toggleFieldVisibility(): void;
  toggleRobotVisibility(): void;
  toggleBallVisibility(): void;
  toggleParticleVisibility(): void;
  toggleGoalVisibility(): void;
  toggleFieldLinePointsVisibility(): void;
  toggleFieldIntersectionsVisibility(): void;
  toggleWalkToDebugVisibility(): void;
  toggleBoundedBoxVisibility(): void;
}

const MenuItem = (props: { label: string; onClick(): void; isVisible: boolean }) => {
  return (
    <li className="flex m-0 p-0">
      <button className="px-4" onClick={props.onClick}>
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
  const { Menu, model, controller } = props;
  return (
    <Menu>
      <ul className="flex h-full items-center">
        <li className="flex px-4">
          <Button className="px-7" onClick={props.onHawkEyeClick}>
            Hawk Eye
          </Button>
        </li>
        <li className="flex px-4">
          <FieldDimensionSelector controller={controller} model={model} />
        </li>
        <MenuItem label="Grid" isVisible={model.gridVisible} onClick={props.toggleGridVisibility} />
        <MenuItem label="Field" isVisible={model.fieldVisible} onClick={props.toggleFieldVisibility} />
        <MenuItem label="Robots" isVisible={model.robotVisible} onClick={props.toggleRobotVisibility} />
        <MenuItem label="Balls" isVisible={model.ballVisible} onClick={props.toggleBallVisibility} />
        <MenuItem label="Particles" isVisible={model.particlesVisible} onClick={props.toggleParticleVisibility} />
        <MenuItem label="Goals" isVisible={model.goalsVisible} onClick={props.toggleGoalVisibility} />
        <MenuItem
          label="Field Line Points"
          isVisible={model.fieldLinePointsVisible}
          onClick={props.toggleFieldLinePointsVisibility}
        />
        <MenuItem
          label="Field Intersections"
          isVisible={model.fieldIntersectionsVisible}
          onClick={props.toggleFieldIntersectionsVisibility}
        />
        <MenuItem label="Walk Path" isVisible={model.walkToDebugVisible} onClick={props.toggleWalkToDebugVisibility} />
        <MenuItem label="Bounded Box" isVisible={model.boundedBoxVisible} onClick={props.toggleBoundedBoxVisibility} />
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
    <div
      className={
        "bg-black/30 rounded-md text-white p-4 text-center absolute bottom-8 left-8 right-8 text-lg font-bold flex justify-between"
      }
    >
      <span className="text-left w-1/3">&#160;</span>
      <span className="w-1/3">{target}</span>
      <span className="text-right w-1/3">{viewModeString(props.model.viewMode)}</span>
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

interface RobotRenderProps {
  robot: LocalisationRobotModel;
  model: LocalisationModel;
}

const RobotComponents: React.FC<RobotRenderProps> = observer(({ robot, model }) => {
  if (!robot.visible) return null;

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
      {model.robotVisible &&
        model.robots.map((robotModel) => {
          return robotModel.visible && <MujocoRobot key={robotModel.id} model={robotModel} />;
        })}
      {model.fieldLinePointsVisible && <FieldLinePoints model={model} />}
      {model.ballVisible && <Balls model={model} />}
      {model.fieldIntersectionsVisible && <FieldIntersections model={model} />}
      {model.particlesVisible && <Particles model={model} />}
      {model.goalVisible && <Goals model={model} />}
      <Robots model={model} />
    </object3D>
  );
});

const LocalisationViewModel: React.FC<{ model: LocalisationModel }> = observer(({ model }) => (
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

    {model.robotVisible && model.robots.map((robot) => <RobotComponents key={robot.id} robot={robot} model={model} />)}
  </object3D>
));

const Balls = ({ model }: { model: LocalisationModel }) => {
  return (
    <>
      {model.robots.map(
        (robot) =>
          robot.visible &&
          robot.rBFf && (
            <mesh position={robot.rBFf.toArray()} scale={[robot.rBFf.z, robot.rBFf.z, robot.rBFf.z]} key={robot.id}>
              <sphereBufferGeometry args={[1, 32, 32]} /> {/* Increased detail for the texture */}
              <meshStandardMaterial map={soccerBallTexture} />
            </mesh>
          ),
      )}
    </>
  );
};

const Goals = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible &&
        robot.rGFf && (
          <object3D key={robot.id}>
            {robot.rGFf.map((goal, i) => {
              return (
                <mesh
                  key={String(i)}
                  position={goal.bottom.add(new Vector3(0, 0, goal.top.z / 2)).toArray()}
                  rotation={[Math.PI / 2, 0, 0]}
                >
                  <cylinderBufferGeometry args={[0.05, 0.05, goal.top.z, 20]} />
                  <meshStandardMaterial color="magenta" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);

const Robots = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible &&
        robot.robots && (
          <object3D key={robot.id}>
            {robot.rRFf.map((r, i) => {
              return (
                <mesh key={String(i)} position={r.add(new Vector3(0, 0, 0.4)).toArray()} rotation={[Math.PI / 2, 0, 0]}>
                  <cylinderBufferGeometry args={[0.1, 0.1, 0.8, 20]} />
                  <meshStandardMaterial color="orange" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);

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
const MujocoRobot = ({ model }: { model: LocalisationRobotModel }) => {
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

  const Hwt = model.Htw_mujoco.invert();

  const position = Hwt.decompose().translation;
  const rotation = Hwt.decompose().rotation;
  const motors = model.motors_mujoco;

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
    color: "#111111",
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
