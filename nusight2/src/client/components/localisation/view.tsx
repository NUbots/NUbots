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
  private dropdownToggle = (
    <button className="inline-flex flex-col items-center justify-center bg-transparent px-3 h-[60px]">
      <Icon size={28} className="mt-1 -ml-5">
        tune
      </Icon>
      <span className="text-[0.7rem]">Field Type</span>
    </button>
  );

  render(): JSX.Element {
    return (
      <EnhancedDropdown dropdownToggle={this.dropdownToggle}>
        <div className="bg-auto-surface-2 shadow-md text-auto-primary">
          {FieldDimensionOptions.map((option) => (
            <div
              key={option.value}
              className="flex items-center p-3 cursor-pointer hover:bg-auto-contrast-1 transition-colors"
              role="menuitem"
              tabIndex={0}
              onClick={() => this.props.controller.setFieldDimensions(option.value, this.props.model)}
            >
              <Icon size={20} className="mr-3">
                {this.props.model.field.fieldType === option.value ? "check_circle" : "radio_button_unchecked"}
              </Icon>
              <span className="text-sm font-medium">{option.label}</span>
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

          {/* Right-side visibility controls panel */}
          <VisibilityPanel
            model={this.props.model}
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
          />
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
    <button
      className={`
        w-full flex items-center gap-2 px-2.5 py-1.5 rounded-lg text-sm font-medium
        transition-all duration-150 ease-out
        ${
          props.isVisible
            ? "bg-auto-primary/20 text-auto-primary border border-auto-primary/30"
            : "bg-auto-surface-2 text-auto-on-surface border border-auto-outline hover:bg-auto-surface-3"
        }
        focus:outline-none focus:ring-1 focus:ring-auto-primary
        active:scale-[0.98]
      `}
      onClick={props.onClick}
    >
      <div
        className={`
        flex items-center justify-center w-3.5 h-3.5 rounded
        ${props.isVisible ? "bg-auto-primary/30 text-auto-primary" : "bg-auto-surface-3 text-auto-on-surface/60"}
      `}
      >
        <Icon size={10}>{props.isVisible ? "visibility" : "visibility_off"}</Icon>
      </div>
      <span className="flex-1 text-left">{props.label}</span>
      <div
        className={`
        w-1 h-1 rounded-full transition-colors
        ${props.isVisible ? "bg-auto-primary" : "bg-auto-on-surface/40"}
      `}
      />
    </button>
  );
};

const LocalisationMenuBar = observer((props: LocalisationMenuBarProps) => {
  const { Menu, model, controller } = props;

  return (
    <Menu>
      <div className="flex min-h-[60px] h-auto flex-wrap items-center bg-auto-surface-1">
        <div className="flex-1 pr-4">
          <div className="flex items-center gap-4">
            {/* Hawk Eye Button - styled like robot selector */}
            <div className="flex relative">
              <button
                className="inline-flex flex-col items-center justify-center bg-transparent px-3 h-[60px]"
                onClick={props.onHawkEyeClick}
              >
                <Icon size={28} className="mt-1 -ml-5 text-blue-500">
                  visibility
                </Icon>
                <span className="text-[0.7rem]">Hawk Eye</span>
              </button>
            </div>

            {/* Field Type Selector - styled like robot selector */}
            <FieldDimensionSelector controller={controller} model={model} />

            {/* Dashboard Toggle - styled like NBS scrubber */}
            <div className="flex relative">
              <button
                className={`
                  inline-flex flex-col items-center justify-center bg-transparent px-3 h-[60px]
                  ${model.dashboard.visible ? "text-green-600" : "text-gray-600"}
                `}
                onClick={props.toggleDashboardVisibility}
              >
                <Icon size={28} className="mt-1 -ml-5">
                  dashboard
                </Icon>
                <span className="text-[0.7rem]">Dashboard</span>
              </button>
              <button
                className="absolute right-0 top-0 mt-1.5 mr-2.5 px-0.5 hover:bg-auto-contrast-1 h-8 rounded inline-flex items-center"
                onClick={props.toggleDashboardVisibility}
              >
                <Icon size={20} weight="500" className="transition-transform">
                  {model.dashboard.visible ? "expand_less" : "expand_more"}
                </Icon>
              </button>
            </div>
          </div>
        </div>
      </div>
      <VisibilityPanel
        model={model}
        controller={controller}
        onHawkEyeClick={props.onHawkEyeClick}
        toggleGridVisibility={props.toggleGridVisibility}
        toggleFieldVisibility={props.toggleFieldVisibility}
        toggleRobotVisibility={props.toggleRobotVisibility}
        toggleBallVisibility={props.toggleBallVisibility}
        toggleParticleVisibility={props.toggleParticleVisibility}
        toggleGoalVisibility={props.toggleGoalVisibility}
        toggleFieldLinePointsVisibility={props.toggleFieldLinePointsVisibility}
        toggleFieldIntersectionsVisibility={props.toggleFieldIntersectionsVisibility}
        toggleWalkToDebugVisibility={props.toggleWalkToDebugVisibility}
        toggleBoundedBoxVisibility={props.toggleBoundedBoxVisibility}
        toggleDashboardVisibility={props.toggleDashboardVisibility}
      />
    </Menu>
  );
});

// Right-side visibility toggle panel
const VisibilityPanel = observer((props: LocalisationMenuBarProps) => {
  const { model } = props;

  const toggleGroups = [
    {
      title: "Field",
      buttons: [
        { label: "Field", isVisible: model.fieldVisible, onClick: props.toggleFieldVisibility },
        { label: "Grid", isVisible: model.gridVisible, onClick: props.toggleGridVisibility },
        { label: "Lines", isVisible: model.fieldLinePointsVisible, onClick: props.toggleFieldLinePointsVisibility },
        {
          label: "Intersections",
          isVisible: model.fieldIntersectionsVisible,
          onClick: props.toggleFieldIntersectionsVisibility,
        },
      ],
    },
    {
      title: "Objects",
      buttons: [
        { label: "Robots", isVisible: model.robotVisible, onClick: props.toggleRobotVisibility },
        { label: "Balls", isVisible: model.ballVisible, onClick: props.toggleBallVisibility },
        { label: "Goals", isVisible: model.goalsVisible, onClick: props.toggleGoalVisibility },
      ],
    },
    {
      title: "Debug",
      buttons: [
        { label: "Particles", isVisible: model.particlesVisible, onClick: props.toggleParticleVisibility },
        { label: "Walk Path", isVisible: model.walkToDebugVisible, onClick: props.toggleWalkToDebugVisibility },
        { label: "Bounding Box", isVisible: model.boundedBoxVisible, onClick: props.toggleBoundedBoxVisibility },
      ],
    },
  ];

  return (
    <div className="fixed top-32 right-4 w-56 bg-auto-surface-1 rounded-lg shadow-lg border border-auto-outline z-40">
      <div className="p-3">
        <h3 className="text-sm font-semibold text-auto-on-surface mb-2">Visibility Controls</h3>
        <div className="space-y-3">
          {toggleGroups.map((group) => (
            <div key={group.title}>
              <h4 className="text-xs font-semibold text-auto-on-surface/70 uppercase tracking-wider mb-1.5">
                {group.title}
              </h4>
              <div className="space-y-1">
                {group.buttons.map((button) => (
                  <MenuItem
                    key={button.label}
                    label={button.label}
                    isVisible={button.isVisible}
                    onClick={button.onClick}
                  />
                ))}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
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
        "bg-black/30 rounded-md text-white p-2 text-center absolute bottom-6 left-6 text-sm font-medium flex flex-col gap-1 w-fit"
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
