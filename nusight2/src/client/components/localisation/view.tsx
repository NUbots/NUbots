import React, { ComponentType, PropsWithChildren } from "react";
import { reaction } from "mobx";
import { disposeOnUnmount, observer } from "mobx-react";
import { now } from "mobx-utils";

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
import { FieldLinePoints } from "./r3f_components/field_line_points/view";
import { GridView } from "./r3f_components/grid/view";
import { LocalisedGoals } from "./r3f_components/localised_goals/view";
import { LocalisedRobots } from "./r3f_components/localised_robots/view";
import { Nugus } from "./r3f_components/nugus/view";
import { Particles } from "./r3f_components/particles/view";
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
    <object3D key={robot.id}>
      <Nugus model={robot} />

      {model.fieldLinePointsVisible && <FieldLinePoints points={robot.rPFf} color="blue" size={0.02} />}

      {model.ballVisible && robot.rBFf && <Ball position={robot.rBFf.toArray()} scale={robot.rBFf.z} />}

      {model.goalsVisible && <LocalisedGoals rGFf={robot.rGFf} />}

      <LocalisedRobots id={robot.id} rRFf={robot.rRFf} />

      {model.fieldIntersectionsVisible && robot.fieldIntersections && (
        <FieldIntersections intersections={robot.fieldIntersections} />
      )}

      {model.walkToDebugVisible && robot.Hfd && robot.Hfr && (
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

      {model.particlesVisible && <Particles particles={robot.particles} />}

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

export default LocalisationView;
