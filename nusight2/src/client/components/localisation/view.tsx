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
import { LocalisationModel, ViewMode } from "./model";
import { LocalisationNetwork } from "./network";
import { AssociationLines } from "./r3f_components/association_lines";
import { Ball } from "./r3f_components/ball";
import { BoundingBox } from "./r3f_components/bounding_box/view";
import { FieldView } from "./r3f_components/field/view";
import { FieldIntersections } from "./r3f_components/field_intersections";
import { FieldObjects } from "./r3f_components/field_objects";
import { FieldPoints } from "./r3f_components/field_points";
import { GridView } from "./r3f_components/grid";
import { Nugus } from "./r3f_components/nugus";
import { PurposeLabel } from "./r3f_components/purpose_label";
import { SkyboxView } from "./r3f_components/skybox/view";
import { WalkPathGoal } from "./r3f_components/walk_path_goal";
import { WalkPathVisualiser } from "./r3f_components/walk_path_visualiser";
import { WalkTrajectory } from "./r3f_components/walk_trajectory";
import { WalkTrajectoryHistory } from "./r3f_components/walk_trajectory_history";
import { LocalisationRobotModel } from "./robot_model";

import { RobotPanel } from "./robot_panel/view";
import { RobotPanelViewModel } from "./robot_panel/view_model";
import { DashboardModel } from "./model";
import { DashboardFieldView } from "./field/view";

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
              className={`flex p-2 ${this.props.model.field.fieldType === option.value
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

        <Dashboard
          controller={this.props.controller}
          Field={() => <DashboardFieldView model={this.props.model.dashboard.field}/>}
          model={this.props.model.dashboard}
          network={this.props.network}
        />
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
    <object3D key={robot.id}>
      <Nugus model={robot} />

      {model.fieldLinePointsVisible && <FieldPoints points={robot.rPFf} color={"blue"} size={0.02} />}
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
          min_align_radius={robot.min_align_radius}
          max_align_radius={robot.max_align_radius}
          min_angle_error={robot.min_angle_error}
          max_angle_error={robot.max_angle_error}
          angle_to_final_heading={robot.angle_to_final_heading}
          velocity_target={robot.velocity_target}
        />
      )}

      {robot.Hft && robot.purpose && (
        <PurposeLabel
          Hft={robot.Hft}
          player_id={robot.player_id}
          backgroundColor={robot.color}
          purpose={robot.purpose}
          cameraPitch={model.camera.pitch}
          cameraYaw={model.camera.yaw}
        />
      )}

      {model.walkToDebugVisible && robot.Hfd && <WalkPathGoal Hfd={robot.Hfd} Hft={robot.Hft} motors={robot.motors} />}

      {robot.torso_trajectory && robot.swing_foot_trajectory && (
        <WalkTrajectory
          torso_trajectory={robot.torso_trajectoryF}
          swing_foot_trajectory={robot.swing_foot_trajectoryF}
          color={"#ffa500"}
        />
      )}

      {robot.trajectory_history.length > 0 && <WalkTrajectoryHistory trajectories={robot.trajectory_history} />}

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
      <pointLight color="white" />
    </PerspectiveCamera>
    <SkyboxView model={model.skybox} />
    <hemisphereLight args={["#fff", "#fff", 0.6]} />

    {model.fieldVisible && <FieldView model={model.field} />}
    {model.gridVisible && <GridView />}

    {model.robotVisible && model.robots.map((robot) => <RobotComponents key={robot.id} robot={robot} model={model} />)}
  </object3D>
));

type DashboardProps = {
  controller: LocalisationController;
  Field: ComponentType;
  model: DashboardModel;
  network: LocalisationNetwork;
};

@observer
export class Dashboard extends Component<DashboardProps> {
  componentWillUnmount(): void {
    this.props.network.destroy();
  }

  render() {
    const { model } = this.props;
    const showPanels = model.robots.some((robot) => robot.enabled);
    const Field = this.props.Field;
    return (
      <div className="flex flex-col w-full h-full">
        <div className="flex flex-col flex-1 bg-auto-surface-0 border-t border-auto h-full">
          <div className="flex-1 relative h-full">
            <Button className="mt-5 ml-5 z-10 relative" onClick={this.onToggleOrientationClick}>Flip Orientation</Button>
            <Field />
          </div>
          {showPanels && (
            <div className="flex p-2">
              {model.robots.map((robot) => {
                const model = RobotPanelViewModel.of(robot);
                return (
                  robot.enabled && (
                    <div className="rounded-sm shadow-md flex-1 ml-2 overflow-hidden first:ml-0" key={robot.id}>
                      <RobotPanel
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

  private onToggleOrientationClick = () => {
    const { controller, model } = this.props;
    controller.toggleOrientation(model);
  };
}

export default LocalisationView;
