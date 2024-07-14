import React from "react";
import { PropsWithChildren } from "react";
import { ComponentType } from "react";
import { reaction } from "mobx";
import { observer } from "mobx-react";
import { disposeOnUnmount } from "mobx-react";
import { now } from "mobx-utils";
import * as THREE from "three";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry.js";
import { FontLoader } from "three/examples/jsm/loaders/FontLoader.js";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { Vector3 } from "../../../shared/math/vector3";
import { Button } from "../button/button";
import { dropdownContainer } from "../dropdown_container/view";
import { Icon } from "../icon/view";
import { PerspectiveCamera } from "../three/three_fiber";
import { ThreeFiber } from "../three/three_fiber";

import { LocalisationController } from "./controller";
import { FieldView } from "./field/view";
import roboto from "./fonts/Roboto Medium_Regular.json";
import { GridView } from "./grid/view";
import { LocalisationModel } from "./model";
import { ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { LocalisationRobotModel } from "./robot_model";
import { SkyboxView } from "./skybox/view";
import { RobotPanel } from "./robot_panel/view";
import { RobotPanelViewModel } from "./robot_panel/view_model";

type LocalisationViewProps = {
  controller: LocalisationController;
  Menu: ComponentType<{}>;
  model: LocalisationModel;
  network: LocalisationNetwork;
};

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

// Ball texture obtained from https://katfetisov.wordpress.com/2014/08/08/freebies-football-textures/
const textureLoader = new THREE.TextureLoader();
const soccerBallTexture = textureLoader.load("/images/ball_texture.png");

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
        ></LocalisationMenuBar>
        <div className="flex w-full h-full">
          <div className="flex-grow relative border-t border-auto">
            <ThreeFiber ref={this.canvas} onClick={this.onClick}>
              <LocalisationViewModel model={this.props.model} />
            </ThreeFiber>
            <StatusBar model={this.props.model} />
          </div>
          <RobotPanels model={this.props.model} />
        </div>
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
        <MenuItem label="Goals" isVisible={model.goalVisible} onClick={props.toggleGoalVisibility} />
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
      </ul>
    </Menu>
  );
});

interface StatusBarProps {
  model: LocalisationModel;
}

const RobotPanels = observer(({ model }: { model: LocalisationModel }) => {
  return (
    <div className="absolute right-0 flex flex-col w-1/5 bg-black/30 overflow-hidden first:ml gap-2 p-2">
      {model.robots.map((robot) => {
        const model = RobotPanelViewModel.of(robot);
        console.log(model);
        return (
          <div className="" key={robot.id}>
            <RobotPanel
              connected={true}
              batteryValue={model.batteryValue}
              lastCameraImage={model.lastCameraImage}
              lastSeenBall={model.lastSeenBall}
              lastSeenGoal={model.lastSeenGoal}
              mode={model.mode}
              penalised={model.penalised}
              penalty={model.penalty}
              phase={model.phase}
              title={model.title}
              walkCommand={new Vector3(0, 0, 0)}
            />
          </div>
        );
      })}
    </div>
  );
});

const StatusBar = observer((props: StatusBarProps) => {
  const target =
    props.model.viewMode !== ViewMode.FreeCamera && props.model.target ? props.model.target.name : "No Target";
  return (
    <div
      className={
        "bg-black/30 rounded-md text-white p-4 text-center absolute bottom-8 left-8 right-[70%] text-lg font-bold flex justify-between"
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
        model.robots
          .filter((robotModel) => robotModel.visible)
          .map((robotModel) => <Robot key={robotModel.id} model={robotModel} />)}
      {model.fieldLinePointsVisible && <FieldLinePoints model={model} />}
      {model.ballVisible && <Balls model={model} />}
      {model.fieldIntersectionsVisible && <FieldIntersections model={model} />}
      {model.particlesVisible && <Particles model={model} />}
      {model.goalVisible && <Goals model={model} />}
      {model.robots
        .filter((robot) => robot.visible && robot.Hfd)
        .map((robot) => (
          <WalkPathVisualiser key={robot.id} model={robot} />
        ))}
      {model.robots
        .filter((robot) => robot.visible && robot.Hft && robot.purpose)
        .map((robot) => (
          <PurposeLabel
            key={robot.id}
            robotModel={robot}
            cameraPitch={model.camera.pitch}
            cameraYaw={model.camera.yaw}
          />
        ))}
      {model.robots
        .filter((robot) => robot.visible && robot.Hfd)
        .map((robot) => (
          <WalkPathGoal key={robot.id} model={robot} />
        ))}
      <Robots model={model} />
    </object3D>
  );
});

const WalkPathVisualiser = ({ model }: { model: LocalisationRobotModel }) => {
  if (!model.Hfd || !model.Hfr) {
    return null;
  }
  const rDFf = model.Hfd?.decompose().translation;
  const rTFf = model.Hft.decompose().translation;
  const robot_rotation = new THREE.Euler().setFromQuaternion(model.Hft.decompose().rotation.toThree(), "XYZ");
  const target_rotation = new THREE.Euler().setFromQuaternion(model.Hfd.decompose().rotation.toThree(), "XYZ");
  const min_align_radius = model.min_align_radius;
  const max_align_radius = model.max_align_radius;
  const min_angle_error = model.min_angle_error;
  const max_angle_error = model.max_angle_error;
  const angle_to_final_heading = model.angle_to_final_heading;
  const Rfr = new THREE.Quaternion(
    model.Hfr?.decompose().rotation.x,
    model.Hfr?.decompose().rotation.y,
    model.Hfr?.decompose().rotation.z,
    model.Hfr?.decompose().rotation.w,
  );
  const vRf = model.velocity_target.toThree().applyQuaternion(Rfr);

  const velocity_direction = Math.atan2(vRf.y, vRf.x);
  const speed = Math.sqrt(vRf.x ** 2 + vRf.y ** 2) * 1.5;

  const arrowGeometry = (length: number) => {
    const arrowShape = new THREE.Shape();

    arrowShape.moveTo(0, -0.01);
    arrowShape.lineTo(0, 0.01);
    arrowShape.lineTo(length * 0.7, 0.01);
    arrowShape.lineTo(length * 0.7, 0.02);
    arrowShape.lineTo(length, 0);
    arrowShape.lineTo(length * 0.7, -0.02);
    arrowShape.lineTo(length * 0.7, -0.01);
    arrowShape.lineTo(0, -0.01);

    const geometry = new THREE.ShapeGeometry(arrowShape);

    return geometry;
  };

  return (
    <object3D>
      <mesh position={[rDFf?.x, rDFf?.y, 0.005]}>
        <circleBufferGeometry args={[min_align_radius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.006]}>
        <circleBufferGeometry args={[max_align_radius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.007]} rotation={[0, 0, target_rotation.z - 0.5 * min_angle_error]}>
        <circleBufferGeometry args={[max_align_radius, 40, 0, min_angle_error]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.008]} rotation={[0, 0, target_rotation.z - 0.5 * max_angle_error]}>
        <circleBufferGeometry args={[max_align_radius, 40, 0, max_angle_error]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.009]}>
        <mesh geometry={arrowGeometry(max_align_radius)} rotation={[0, 0, robot_rotation.z]}>
          <meshBasicMaterial color="rgb(255, 255, 255)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.011]}>
        <mesh geometry={arrowGeometry(speed)} rotation={[0, 0, velocity_direction]}>
          <meshBasicMaterial color="rgb(0, 255, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.011]}>
        <mesh geometry={arrowGeometry(min_align_radius)} rotation={[0, 0, robot_rotation.z + angle_to_final_heading]}>
          <meshBasicMaterial color="rgb(255, 0, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
    </object3D>
  );
};

const PurposeLabel = ({
  robotModel,
  cameraPitch,
  cameraYaw,
}: {
  robotModel: LocalisationRobotModel;
  cameraPitch: number;
  cameraYaw: number;
}) => {
  const rTFf = robotModel.Hft.decompose().translation;
  const textGeometry = (x: string) => {
    const font = new FontLoader().parse(roboto);
    return new TextGeometry(x, {
      font: font,
      size: 0.1,
      height: 0,
    }).center();
  };

  const textBackdropGeometry = (width: number, height: number) => {
    const shape = new THREE.Shape();
    width += 0.1;
    height += 0.1;
    const radius = 0.05;
    const x = width * -0.5;
    const y = height * -0.5;

    shape.moveTo(x, y + radius);
    shape.lineTo(x, y + height - radius);
    shape.quadraticCurveTo(x, y + height, x + radius, y + height);
    shape.lineTo(x + width - radius, y + height);
    shape.quadraticCurveTo(x + width, y + height, x + width, y + height - radius);
    shape.lineTo(x + width, y + radius);
    shape.quadraticCurveTo(x + width, y, x + width - radius, y);
    shape.lineTo(x + radius, y);
    shape.quadraticCurveTo(x, y, x, y + radius);

    const geometry = new THREE.ShapeGeometry(shape);

    return geometry;
  };

  const labelTextGeometry = textGeometry(robotModel.purpose);
  labelTextGeometry.computeBoundingBox();
  const textWidth = labelTextGeometry.boundingBox
    ? labelTextGeometry.boundingBox.max.x - labelTextGeometry.boundingBox.min.x
    : 0;
  const textHeight = labelTextGeometry.boundingBox
    ? labelTextGeometry.boundingBox.max.y - labelTextGeometry.boundingBox.min.y
    : 0;
  const backdropGeometry = textBackdropGeometry(textWidth, textHeight);

  return (
    <object3D
      position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.6]}
      rotation={[Math.PI / 2 + cameraPitch, 0, -Math.PI / 2 + cameraYaw, "ZXY"]}
    >
      <mesh position={[0, 0, 0.001]} geometry={textGeometry(robotModel.purpose)}>
        <meshBasicMaterial color="white" transparent opacity={1} />
      </mesh>
      <mesh geometry={backdropGeometry}>
        <meshBasicMaterial color="black" transparent opacity={0.5} />
      </mesh>
    </object3D>
  );
};

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

const FieldIntersections = ({ model }: { model: LocalisationModel }) => {
  return (
    <>
      {model.robots.map(
        (robot) =>
          robot.visible && (
            <object3D key={robot.id}>
              {robot.fieldIntersectionsF?.map((intersection) => {
                const createShapeForIntersection = (intersectionType: string, position: Vector3) => {
                  const basePosition = position.add(new Vector3(0.1, 0.1, 0)).toArray();
                  switch (intersectionType) {
                    case "L_INTERSECTION":
                      return (
                        <>
                          <mesh position={intersection.position.add(new Vector3(0, 0, 0.01)).toArray()}>
                            <circleBufferGeometry args={[0.04, 20]} />
                            <meshBasicMaterial color="red" />
                          </mesh>
                          <mesh position={[basePosition[0], basePosition[1] - 0.05, basePosition[2]]}>
                            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                          <mesh position={[basePosition[0] - 0.04, basePosition[1], basePosition[2]]}>
                            <boxBufferGeometry args={[0.02, 0.1, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                        </>
                      );
                    case "T_INTERSECTION":
                      return (
                        <>
                          <mesh position={intersection.position.add(new Vector3(0, 0, 0.01)).toArray()}>
                            <circleBufferGeometry args={[0.04, 20]} />
                            <meshBasicMaterial color="red" />
                          </mesh>
                          <mesh position={[basePosition[0], basePosition[1] + 0.05, basePosition[2]]}>
                            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                          <mesh position={[basePosition[0], basePosition[1], basePosition[2]]}>
                            <boxBufferGeometry args={[0.02, 0.1, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                        </>
                      );
                    case "X_INTERSECTION":
                      return (
                        <>
                          <mesh position={intersection.position.add(new Vector3(0, 0, 0.01)).toArray()}>
                            <circleBufferGeometry args={[0.04, 20]} />
                            <meshBasicMaterial color="red" />
                          </mesh>
                          <mesh
                            position={[basePosition[0], basePosition[1], basePosition[2]]}
                            rotation={[0, 0, Math.PI / 4]}
                          >
                            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                          <mesh
                            position={[basePosition[0], basePosition[1], basePosition[2]]}
                            rotation={[0, 0, -Math.PI / 4]}
                          >
                            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
                            <meshBasicMaterial color="black" />
                          </mesh>
                        </>
                      );
                    default:
                      return null;
                  }
                };
                return createShapeForIntersection(intersection.type, intersection.position);
              })}
            </object3D>
          ),
      )}
    </>
  );
};

const Particles = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible && (
          <object3D key={robot.id}>
            {robot.particles.particle.map((particle, i) => {
              return (
                <mesh key={String(i)} position={new Vector3(particle.x, particle.y, 0.005).toArray()}>
                  <circleBufferGeometry args={[0.02, 20]} />
                  <meshBasicMaterial color="red" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);

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

const WalkPathGoal = ({ model }: { model: LocalisationRobotModel }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(nugusUrdfPath, (robot: URDFRobot) => {
      if (robotRef.current) {
        robotRef.current.add(robot);
      }
    });
  }, []);

  const rDFf = model.Hfd?.decompose().translation;
  const rTFf = model.Hft.decompose().translation;
  const Rfd_quat = new THREE.Quaternion(
    model.Hfd?.decompose().rotation.x,
    model.Hfd?.decompose().rotation.y,
    model.Hfd?.decompose().rotation.z,
    model.Hfd?.decompose().rotation.w,
  );
  const Rft_quat = new THREE.Quaternion(
    model.Hft.decompose().rotation.x,
    model.Hft.decompose().rotation.y,
    model.Hft.decompose().rotation.z,
    model.Hft.decompose().rotation.w,
  );

  // Get euler angles from quaternion
  const Rfz_euler = new THREE.Euler().setFromQuaternion(Rfd_quat, "ZYX");
  const Rft_euler = new THREE.Euler().setFromQuaternion(Rft_quat, "ZYX");
  // Fuse the euler angles into a single quaternion
  const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(Rft_euler.x, Rft_euler.y, Rfz_euler.z, "ZYX"));
  const position = new THREE.Vector3(rDFf?.x, rDFf?.y, rTFf.z);

  const motors = model.motors;

  // Update the position of the robot to match the walk path goal
  React.useEffect(() => {
    if (robotRef.current) {
      robotRef.current.position.set(position.x, position.y, position.z);
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
      robotRef.current.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          // Set opacity for all mesh children
          child.material.transparent = true;
          // Red
          child.material.color = "rgb(0, 100, 100)";
          child.material.opacity = 0.2;
        }
      });
    }
  });

  return <object3D ref={robotRef} />;
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
