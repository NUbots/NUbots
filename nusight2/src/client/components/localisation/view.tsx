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
import { FieldPoints } from "./r3f_components/field_points/view";
import { GridView } from "./r3f_components/grid/view";
import { FieldObjects } from "./r3f_components/field_objects/view";
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
          Menu={this.props.Menu}
          controller={this.props.controller}
          onHawkEyeClick={this.onHawkEyeClick}
        ></LocalisationMenuBar>
        <div className="border-t border-auto grid grid-cols-4 flex-grow relative">
          <div className="bg-red-500 col-span-3 flex-grow relative  ">
            <ThreeFiber ref={this.canvas} onClick={this.onClick}>
              <LocalisationViewModel model={this.props.model} />
            </ThreeFiber>
            <StatusBar model={this.props.model} />
          </div>
          <div className="bg-auto-surface-2 relative p-4">
            sample
            {/* <CheckboxTree /> */}
          </div>
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
}

interface LocalisationMenuBarProps {
  Menu: ComponentType<PropsWithChildren>;
  controller: LocalisationController;

  onHawkEyeClick(): void;
}

const LocalisationMenuBar = observer((props: LocalisationMenuBarProps) => {
  const { Menu, controller } = props;
  return (
    <Menu>
      <ul className="flex h-full items-center">
        <li className="flex px-4">
          <Button className="px-7" onClick={props.onHawkEyeClick}>
            Hawk Eye
          </Button>
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
    <div
      className={
        "bg-black/30 rounded-md text-white p-4 text-center absolute bottom-4 left-4 right-4 text-lg font-bold flex justify-between"
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

      {model.fieldLinePointsVisible && <FieldPoints points={robot.rPFf} color={robot.color} size={0.02} />}
      {model.particlesVisible && <FieldPoints points={robot.particles} color={robot.color} size={0.02} />}

      {model.ballVisible && robot.rBFf && <Ball position={robot.rBFf.toArray()} scale={robot.rBFf.z} />}

      {model.goalsVisible &&
        <FieldObjects
          objects={robot.rGFf.map(goal => ({
            position: goal.bottom,
            height: goal.top.z,
          }))}
          defaultRadius={0.05}
          defaultColor="magenta"
        />
      }

      <FieldObjects
        objects={robot.rRFf.map(r => ({
          position: r,
        }))}
        defaultHeight={0.8}
        defaultRadius={0.1}
        defaultColor="orange"
      />

      {model.fieldIntersectionsVisible && robot.fieldIntersections && (
        <FieldIntersections intersections={robot.fieldIntersections} />
      )}

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
