import React, { Component, ComponentType, PropsWithChildren } from "react";
import { reaction } from "mobx";
import { observer } from "mobx-react";
import { now } from "mobx-utils";

import { compose } from "../../../shared/base/compose";
import { Button } from "../button/button";
import { dropdownContainer } from "../dropdown_container/view";
import { Icon } from "../icon/view";
import { PerspectiveCamera, ThreeFiber } from "../three/three_fiber";

import { LocalisationController } from "./controller";
import { DashboardRobotModel } from "./dashboard_components/dashboard_robot/model";
import { DashboardFieldView } from "./dashboard_components/field/view";
import { DashboardRobotPanel } from "./dashboard_components/robot_panel/view";
import { DashboardRobotPanelViewModel } from "./dashboard_components/robot_panel/view_model";
import { LocalisationModel, ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { AssociationLines } from "./r3f_components/association_lines";
import { Ball } from "./r3f_components/ball";
import { BoundingBox } from "./r3f_components/bounding_box/view";
import { FieldView } from "./r3f_components/field/view";
import { FieldIntersections } from "./r3f_components/field_intersections";
import { FieldObjects } from "./r3f_components/field_objects";
import { FieldPoints } from "./r3f_components/field_points";
import { FieldSpheres } from "./r3f_components/field_spheres";
import { GridView } from "./r3f_components/grid";
import { Nugus } from "./r3f_components/nugus";
import { PurposeLabel } from "./r3f_components/purpose_label";
import { SkyboxView } from "./r3f_components/skybox/view";
import { WalkPathGoal } from "./r3f_components/walk_path_goal";
import { WalkPathVisualiser } from "./r3f_components/walk_path_visualiser";
import { WalkTrajectory } from "./r3f_components/walk_trajectory";
import { WalkTrajectoryHistory } from "./r3f_components/walk_trajectory_history";
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
              className="flex items-center p-2 cursor-pointer hover:bg-auto-contrast-1"
              role="menuitem"
              tabIndex={0}
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
      <div className="flex flex-col relative bg-auto-surface-0 overflow-x-hidden w-full">
        <div className="w-full">
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
            toggleDashboardVisibility={this.toggleDashboardVisibility}
          ></LocalisationMenuBar>
        </div>

        <div
          className={`flex-grow relative border-t border-auto${this.props.model.dashboard.visible ? " hidden" : ""}`}
        >
          <ThreeFiber ref={this.canvas} onClick={this.onClick}>
            <LocalisationViewModel model={this.props.model} />
          </ThreeFiber>
          <StatusBar model={this.props.model} />
        </div>

        {this.props.model.dashboard.visible && (
          <DashboardField
            controller={this.props.controller}
            Field={() => <DashboardFieldView model={this.props.model.dashboardField} />}
            model={this.props.model}
          />
        )}

        <DashboardPanel model={this.props.model} robots={this.props.model.dashboardRobots} />
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

  private toggleDashboardVisibility = () => {
    this.props.controller.toggleDashboardVisibility(this.props.model);
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
  toggleDashboardVisibility(): void;
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
      <div className="grid grid-cols-1 md:grid-cols-[auto_1fr] gap-x-[5rem] gap-y-4 px-3 py-3 w-full items-center">
        <div className="flex gap-5 items-center w-fit">
          <Button onClick={props.onHawkEyeClick}>Hawk Eye</Button>
          <FieldDimensionSelector controller={controller} model={model} />
        </div>

        <div className="flex flex-wrap w-fit gap-x-4 gap-y-4">
          {[
            ["Grid", model.gridVisible, props.toggleGridVisibility],
            ["Field", model.fieldVisible, props.toggleFieldVisibility],
            ["Robots", model.robotVisible, props.toggleRobotVisibility],
            ["Balls", model.ballVisible, props.toggleBallVisibility],
            ["Particles", model.particlesVisible, props.toggleParticleVisibility],
            ["Goals", model.goalsVisible, props.toggleGoalVisibility],
            ["Field Line Points", model.fieldLinePointsVisible, props.toggleFieldLinePointsVisibility],
            ["Field Intersections", model.fieldIntersectionsVisible, props.toggleFieldIntersectionsVisibility],
            ["Walk Path", model.walkToDebugVisible, props.toggleWalkToDebugVisibility],
            ["Bounded Box", model.boundedBoxVisible, props.toggleBoundedBoxVisibility],
            ["Dashboard", model.dashboard.visible, props.toggleDashboardVisibility],
          ].map(([label, isVisible, onClick]) => (
            <div key={label as string} className="w-fit max-w-[13rem]">
              <MenuItem label={label as string} isVisible={isVisible as boolean} onClick={onClick as () => void} />
            </div>
          ))}
        </div>
      </div>
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
        "bg-black/30 rounded-md text-white p-4 text-center absolute bottom-8 left-8 text-lg font-bold flex flex-col gap-2 w-fit"
      }
    >
      <span>{target}</span>
      <span>{viewModeString(props.model.viewMode)}</span>
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
    <object3D key={robot.id}>
      <Nugus model={robot} />

      {model.fieldLinePointsVisible && <FieldPoints points={robot.rPFf} color={"blue"} size={0.02} />}
      {model.stellaMapPointsVisible && robot.stellaMapPoints.rPWw_map && <FieldSpheres points={robot.stellaMapPoints.rPWw_map} color={"blue"} size={0.015} />}
      {model.particlesVisible && <FieldPoints points={robot.particles} color={"blue"} size={0.02} />}

      {model.ballVisible && robot.rBFf && <Ball position={robot.rBFf.toArray()} scale={robot.rBFf.z} />}

      {model.goalsVisible && (
        <FieldObjects
          objects={robot.rGFf.map((goal) => ({
            position: goal.bottom,
            height: goal.top.z,
          }))}
          defaultRadius={0.05}
          defaultColor="magenta"
        />
      )}

      <FieldObjects
        objects={robot.rRFf.map((r) => {
          return {
            position: r.position,
            color: r.color,
          };
        })}
        defaultHeight={0.8}
        defaultRadius={0.1}
      />

      {model.fieldIntersectionsVisible && robot.rIFf && <FieldIntersections intersections={robot.rIFf} />}

      {model.fieldIntersectionsVisible && robot.associationLines && <AssociationLines lines={robot.associationLines} />}

      {model.walkToDebugVisible && robot.Hfd && robot.Hfr && robot.Hft && (
        <WalkPathVisualiser
          Hfd={robot.Hfd}
          Hfr={robot.Hfr}
          Hft={robot.Hft}
          minAlignRadius={robot.minAlignRadius}
          maxAlignRadius={robot.maxAlignRadius}
          minAngleError={robot.minAngleError}
          maxAngleError={robot.maxAngleError}
          angleToFinalHeading={robot.angleToFinalHeading}
          velocityTarget={robot.velocityTarget}
        />
      )}

      {robot.Hft && robot.purpose && (
        <PurposeLabel
          Hft={robot.Hft}
          playerId={robot.playerId}
          backgroundColor={robot.color}
          purpose={robot.purpose}
          cameraPitch={model.camera.pitch}
          cameraYaw={model.camera.yaw}
        />
      )}

      {model.walkToDebugVisible && robot.Hfd && <WalkPathGoal Hfd={robot.Hfd} Hft={robot.Hft} motors={robot.motors} />}

      {robot.torsoTrajectory && robot.swingFootTrajectory && (
        <WalkTrajectory
          torsoTrajectory={robot.torsoTrajectoryF}
          swingFootTrajectory={robot.swingFootTrajectoryF}
          color={"#ffa500"}
        />
      )}

      {robot.trajectoryHistory.length > 0 && <WalkTrajectoryHistory trajectories={robot.trajectoryHistory} />}

      {model.boundedBoxVisible && robot.boundingBox && (
        <BoundingBox
          minX={robot.boundingBox.minX}
          maxX={robot.boundingBox.maxX}
          minY={robot.boundingBox.minY}
          maxY={robot.boundingBox.maxY}
          color={robot.color}
        />
      )}
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
      {/* Removed the pointLight that was causing the flashlight effect */}
    </PerspectiveCamera>
    <SkyboxView model={model.skybox} />

    {/* Main directional light matching the skybox sun position */}
    <directionalLight
      position={[
        model.skybox.inclination * 10 - 5,
        model.skybox.azimuth * 10 - 5,
        Math.max(2, model.skybox.inclination * 8),
      ]}
      intensity={1.2}
      color="#ffffff"
      castShadow
      shadow-mapSize-width={2048}
      shadow-mapSize-height={2048}
      shadow-camera-far={20}
      shadow-camera-left={-10}
      shadow-camera-right={10}
      shadow-camera-top={10}
      shadow-camera-bottom={-10}
    />

    {/* Hemisphere light for ambient lighting */}
    <hemisphereLight args={["#87CEEB", "#DEB887", 0.7]} position={[0, 0, 10]} />

    {/* Increased ambient light to ensure consistent illumination throughout the scene */}
    <ambientLight intensity={0.7} color="#ffffff" />

    {/* Fill light from the opposite direction for better depth */}
    <directionalLight position={[5, 5, 3]} intensity={0.4} color="#ffffff" />

    {model.fieldVisible && <FieldView model={model.field} />}
    {model.gridVisible && <GridView />}

    {model.robotVisible && model.robots.map((robot) => <RobotComponents key={robot.id} robot={robot} model={model} />)}
  </object3D>
));

type DashboardFieldProps = {
  controller: LocalisationController;
  Field: ComponentType;
  model: LocalisationModel;
};

@observer
export class DashboardField extends Component<DashboardFieldProps> {
  render() {
    const Field = this.props.Field;
    return (
      <div className="flex flex-col w-full h-full">
        <div className="flex flex-col flex-1 bg-auto-surface-0 border-t border-auto h-full">
          <div className="flex-1 relative h-full">
            <Button className="mt-5 ml-5 z-10 relative" onClick={this.onToggleOrientationClick}>
              Flip Orientation
            </Button>
            <Field />
          </div>
        </div>
      </div>
    );
  }

  private onToggleOrientationClick = () => {
    const { controller, model } = this.props;
    controller.toggleOrientation(model);
  };
}

type DashboardPanelProps = {
  model: LocalisationModel;
  robots: DashboardRobotModel[];
};

@observer
export class DashboardPanel extends Component<DashboardPanelProps> {
  render() {
    const { robots } = this.props;
    const showPanels = robots.some((robot) => robot.enabled);
    return (
      <div className="flex flex-col w-full">
        <div className="flex flex-col flex-1 bg-auto-surface-0 border-t border-auto">
          {showPanels && (
            <div className="flex p-2">
              {robots.map((robot) => {
                const model = DashboardRobotPanelViewModel.of(robot);
                return (
                  robot.enabled && (
                    <div className="rounded-sm shadow-md flex-1 ml-2 overflow-hidden first:ml-0" key={robot.id}>
                      <DashboardRobotPanel
                        connected={model.connected}
                        batteryValue={model.batteryValue}
                        lastCameraImage={model.lastCameraImage}
                        lastSeenBall={model.lastSeenBall}
                        lastSeenGoal={model.lastSeenGoal}
                        mode={model.mode}
                        penalised={model.penalised}
                        penalty={model.penalty}
                        phase={model.phase}
                        title={model.title}
                        walkCommand={model.walkCommand}
                      />
                    </div>
                  )
                );
              })}
            </div>
          )}
        </div>
      </div>
    );
  }
}

export default LocalisationView;
