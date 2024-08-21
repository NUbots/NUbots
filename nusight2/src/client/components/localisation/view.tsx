import React, { useEffect } from "react";
import { observer } from "mobx-react";
import { reaction } from "mobx";
import { now } from "mobx-utils";
import { PerspectiveCamera } from "../three/three_fiber";
import { ThreeFiber } from "../three/three_fiber";
import { dropdownContainer } from "../dropdown_container/view";
import { Button } from "../button/button";
import { LocalisationController } from "./controller";
import { LocalisationModel, ViewMode } from "./model";
import { LocalisationNetwork } from "./network";

import { FieldView } from "./r3f_components/objects/field/view";
import { GridView } from "./r3f_components/objects/grid/view";
import { SkyboxView } from "./r3f_components/objects/skybox/view";
import { URDFNugus } from "./r3f_components/objects/urdf_nugus/view";
import { BoundingBox } from "./r3f_components/objects/bounding_box/view";
import { WalkPathVisualiser } from "./r3f_components/objects/walk_path_visualiser/view";
import { WalkPathGoal } from "./r3f_components/objects/walk_path_goal/view";
import { FieldIntersections } from "./r3f_components/objects/field_intersections/view";
import { LocalisedRobots } from "./r3f_components/objects/localised_robots/view";
import { FieldLinePoints } from "./r3f_components/objects/field_line_points/view";
import { Particles } from "./r3f_components/objects/particles/view";
import { Goals } from "./r3f_components/objects/localised_goals/view";
import { Ball } from "./r3f_components/objects/ball/view";
import { PurposeLabel } from "./r3f_components/objects/purpose_label/view";
import { Icon } from "../icon/view";

const FIELD_DIMENSION_OPTIONS = [
  { label: "Lab", value: "lab" },
  { label: "Robocup", value: "robocup" },
];

interface LocalisationViewProps {
  controller: LocalisationController;
  Menu: React.ComponentType<React.PropsWithChildren>;
  model: LocalisationModel;
  network: LocalisationNetwork;
}

export const LocalisationView: React.FC<LocalisationViewProps> = observer(({ controller, Menu, model, network }) => {
  const canvasRef = React.useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const onPointerLockChange = () => {
      controller.onPointerLockChange(model, canvasRef.current === document.pointerLockElement);
    };

    const onMouseMove = (e: MouseEvent) => {
      controller.onMouseMove(model, e.movementX, e.movementY);
    };

    const onKeyDown = (e: KeyboardEvent) => {
      controller.onKeyDown(model, e.keyCode, {
        shiftKey: e.shiftKey,
        ctrlKey: e.ctrlKey,
      });
    };

    const onKeyUp = (e: KeyboardEvent) => {
      controller.onKeyUp(model, e.keyCode);
    };

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      controller.onWheel(model, e.deltaY);
    };

    document.addEventListener("pointerlockchange", onPointerLockChange);
    document.addEventListener("mousemove", onMouseMove);
    document.addEventListener("keydown", onKeyDown);
    document.addEventListener("keyup", onKeyUp);
    document.addEventListener("wheel", onWheel);

    const dispose = reaction(() => now("frame"), (time: number) => {
      controller.onAnimationFrame(model, time);
    });

    return () => {
      document.removeEventListener("pointerlockchange", onPointerLockChange);
      document.removeEventListener("mousemove", onMouseMove);
      document.removeEventListener("keydown", onKeyDown);
      document.removeEventListener("keyup", onKeyUp);
      document.removeEventListener("wheel", onWheel);
      dispose();
      network.destroy();
    };
  }, [controller, model, network]);

  const onClick = (e: { button: number }) => {
    if (e.button === 0) {
      controller.onLeftClick(model, () => canvasRef.current?.requestPointerLock());
    } else if (e.button === 2) {
      controller.onRightClick(model);
    }
  };


  return (
    <div className="flex flex-grow flex-shrink flex-col relative bg-auto-surface-0">
      <LocalisationMenuBar
        model={model}
        Menu={Menu}
        controller={controller}
        onHawkEyeClick={() => controller.onHawkEyeClick(model)}
        toggleVisibility={{
          grid: () => controller.toggleGridVisibility(model),
          field: () => controller.toggleFieldVisibility(model),
          robot: () => controller.toggleRobotVisibility(model),
          ball: () => controller.toggleBallVisibility(model),
          particle: () => controller.toggleParticlesVisibility(model),
          goal: () => controller.toggleGoalVisibility(model),
          fieldLinePoints: () => controller.toggleFieldLinePointsVisibility(model),
          fieldIntersections: () => controller.toggleFieldIntersectionsVisibility(model),
          walkToDebug: () => controller.toggleWalkToDebugVisibility(model),
          boundedBox: () => controller.toggleBoundedBoxVisibility(model),
        }}
      />
      <div className="flex-grow relative border-t border-auto">
        <ThreeFiber ref={canvasRef} onClick={onClick}>
          <LocalisationViewModel model={model} />
        </ThreeFiber>
      </div>
      <StatusBar model={model} />
    </div>
  );
});

const FieldDimensionSelector: React.FC<{ model: LocalisationModel; controller: LocalisationController }> = observer(
  ({ model, controller }) => {
    const dropdownToggle = <Button>Field Type</Button>;

    return (
      <EnhancedDropdown dropdownToggle={dropdownToggle}>
        <div className="bg-auto-surface-2">
          {FIELD_DIMENSION_OPTIONS.map((option) => (
            <div
              key={option.value}
              className={`flex p-2 ${model.field.fieldType === option.value ? "hover:bg-auto-contrast-1" : "hover:bg-auto-contrast-1"
                }`}
              onClick={() => controller.setFieldDimensions(option.value, model)}
            >
              <Icon size={24}>{model.field.fieldType === option.value ? "check_box" : "check_box_outline_blank"}</Icon>
              <span>{option.label}</span>
            </div>
          ))}
        </div>
      </EnhancedDropdown>
    );
  }
);
const EnhancedDropdown = dropdownContainer();

const MenuItem: React.FC<{ label: string; onClick(): void; isVisible: boolean }> = ({ label, onClick, isVisible }) => (
  <li className="flex m-0 p-0">
    <button className="px-4" onClick={onClick}>
      <div className="flex items-center justify-center">
        <div className="flex items-center rounded">
          <span className="mx-2">{label}</span>
          <Icon size={24}>{isVisible ? "check_box" : "check_box_outline_blank"}</Icon>
        </div>
      </div>
    </button>
  </li>
);

const LocalisationMenuBar: React.FC<{
  Menu: React.ComponentType<React.PropsWithChildren>;
  model: LocalisationModel;
  controller: LocalisationController;
  onHawkEyeClick(): void;
  toggleVisibility: Record<string, () => void>;
}> = observer(({ Menu, model, controller, onHawkEyeClick, toggleVisibility }) => (
  <Menu>
    <ul className="flex h-full items-center">
      <li className="flex px-4">
        <Button className="px-7" onClick={onHawkEyeClick}>
          Hawk Eye
        </Button>
      </li>
      <li className="flex px-4">
        <FieldDimensionSelector controller={controller} model={model} />
      </li>
      <MenuItem label="Grid" isVisible={model.gridVisible} onClick={toggleVisibility.grid} />
      <MenuItem label="Field" isVisible={model.fieldVisible} onClick={toggleVisibility.field} />
      <MenuItem label="Robots" isVisible={model.robotVisible} onClick={toggleVisibility.robot} />
      <MenuItem label="Balls" isVisible={model.ballVisible} onClick={toggleVisibility.ball} />
      <MenuItem label="Particles" isVisible={model.particlesVisible} onClick={toggleVisibility.particle} />
      <MenuItem label="Goals" isVisible={model.goalVisible} onClick={toggleVisibility.goal} />
      <MenuItem label="Field Line Points" isVisible={model.fieldLinePointsVisible} onClick={toggleVisibility.fieldLinePoints} />
      <MenuItem label="Field Intersections" isVisible={model.fieldIntersectionsVisible} onClick={toggleVisibility.fieldIntersections} />
      <MenuItem label="Walk Path" isVisible={model.walkToDebugVisible} onClick={toggleVisibility.walkToDebug} />
      <MenuItem label="Bounded Box" isVisible={model.boundedBoxVisible} onClick={toggleVisibility.boundedBox} />
    </ul>
  </Menu>
));

const StatusBar: React.FC<{ model: LocalisationModel }> = observer(({ model }) => {
  const target =
    model.viewMode !== ViewMode.FreeCamera && model.target ? model.target.name : "No Target";

  return (
    <div className="bg-black/30 rounded-md text-white p-4 text-center absolute bottom-8 left-8 right-8 text-lg font-bold flex justify-between">
      <span className="text-left w-1/3">&#160;</span>
      <span className="w-1/3">{target}</span>
      <span className="text-right w-1/3">{viewModeString(model.viewMode)}</span>
    </div>
  );
});

const viewModeString = (viewMode: ViewMode): string => {
  const modes = {
    [ViewMode.FreeCamera]: "Free Camera",
    [ViewMode.FirstPerson]: "First Person",
    [ViewMode.ThirdPerson]: "Third Person",
  };
  return modes[viewMode] || "Unknown";
};

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
    {model.robotVisible && model.robots.filter(robot => robot.visible).map(robot => (
      <URDFNugus key={robot.id} model={robot} />
    ))}
    {model.fieldLinePointsVisible && model.robots.map((robot) =>
      robot.visible && (
        <FieldLinePoints
          key={robot.id}
          points={robot.rPFf}
          color="blue"
          size={0.02}
        />
      )
    )}

    {model.ballVisible && model.robots.map((robot) =>
      robot.visible && robot.rBFf && (
        <Ball
          key={robot.id}
          position={robot.rBFf.toArray()}
          scale={robot.rBFf.z}
        />
      )
    )}
    {model.fieldIntersectionsVisible && <FieldIntersections model={model} />}
    {model.particlesVisible && <Particles model={model} />}
    {model.goalVisible && <Goals model={model} />}
    {model.walkToDebugVisible && model.robots.filter(robot => robot.visible && robot.Hfd).map(robot => (
      <WalkPathVisualiser key={robot.id} model={robot} />
    ))}
    {model.robots.filter(robot => robot.visible && robot.Hft && robot.purpose).map(robot => (
      <PurposeLabel
        key={robot.id}
        robotModel={robot}
        cameraPitch={model.camera.pitch}
        cameraYaw={model.camera.yaw}
      />
    ))}
    {model.walkToDebugVisible && model.robots.filter(robot => robot.visible && robot.Hfd).map(robot => (
      <WalkPathGoal key={robot.id} model={robot} />
    ))}
    <LocalisedRobots model={model} />
    {model.boundedBoxVisible && model.robots.map(robot => (
      robot.boundingBox && robot.visible && (
        <BoundingBox
          key={robot.id}
          minX={robot.boundingBox.minX}
          maxX={robot.boundingBox.maxX}
          minY={robot.boundingBox.minY}
          maxY={robot.boundingBox.maxY}
          color={robot.color}
        />
      )
    ))}
  </object3D>
));

export default LocalisationView;
