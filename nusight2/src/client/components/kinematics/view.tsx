import React, { PropsWithChildren, useEffect, useRef, useState } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { action } from "mobx";
import { observer } from "mobx-react";
import * as THREE from "three";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { KinematicsController } from "./controller";
import { KinematicsModel } from "./model";
import { Nugus } from "./r3f_components/nugus/view";
import { KinematicsRobotModel } from "./robot_model";

// CameraControls component
const CameraControls = () => {
  const { camera, gl } = useThree();
  const [isDragging, setIsDragging] = useState(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(camera.fov);
  const spherical = useRef(new THREE.Spherical(20, Math.PI / 2.75, Math.PI / 4)).current;

  useEffect(() => {
    camera.position.setFromSpherical(spherical);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
  }, [camera, spherical]);

  useEffect(() => {
    const handleMouseDown = (event: MouseEvent) => {
      setIsDragging(true);
      dragStart.current = { x: event.clientX, y: event.clientY };
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    const handleMouseMove = (event: MouseEvent) => {
      if (!isDragging) return;

      const deltaX = event.clientX - dragStart.current.x;
      const deltaY = event.clientY - dragStart.current.y;
      dragStart.current = { x: event.clientX, y: event.clientY };

      spherical.theta -= deltaX * 0.005;
      spherical.phi -= deltaY * 0.005;

      spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

      camera.position.setFromSpherical(spherical);
      camera.lookAt(0, 0, 0);
    };

    const handleWheel = (event: WheelEvent) => {
      if (event.ctrlKey) {
        event.preventDefault();
        setZoom((prevZoom: number) => {
          const newZoom = Math.min(Math.max(prevZoom + event.deltaY * 0.05, 20), 100);
          return newZoom;
        });
        event.preventDefault();
      }
    };

    gl.domElement.addEventListener("mousedown", handleMouseDown);
    gl.domElement.addEventListener("mousemove", handleMouseMove);
    gl.domElement.addEventListener("mouseup", handleMouseUp);
    gl.domElement.addEventListener("wheel", handleWheel);

    return () => {
      gl.domElement.removeEventListener("mousedown", handleMouseDown);
      gl.domElement.removeEventListener("mousemove", handleMouseMove);
      gl.domElement.removeEventListener("mouseup", handleMouseUp);
      gl.domElement.removeEventListener("wheel", handleWheel);
    };
  }, [camera, gl, isDragging, spherical]);

  useEffect(() => {
    camera.fov = zoom;
    camera.updateProjectionMatrix();
  }, [zoom, camera]);

  return null;
};

// Arrow component
const Arrow = ({
  dir,
  color,
  length,
  thickness,
}: {
  dir: [number, number, number];
  color: number;
  length: number;
  thickness: number;
}) => {
  const shaftLength = length * 0.95;
  const headLength = length * 0.05;

  // Determine rotation for each direction
  const rotation = (() => {
    if (dir[0] === 1) return [0, 0, -Math.PI / 2]; // Rotate X-axis arrow
    if (dir[1] === 1) return [0, 0, 0]; // No rotation for Y-axis arrow
    if (dir[2] === 1) return [Math.PI / 2, 0, 0]; // Rotate Z-axis arrow
    return [0, 0, 0];
  })();

  return (
    <group rotation={rotation}>
      {/* Arrow shaft (cylinder) */}
      <mesh position={[0, shaftLength / 2, 0]}>
        <cylinderGeometry
          args={[thickness, thickness, shaftLength, 32]} // [top radius, bottom radius, height, radial segments]
        />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Arrow head (cone) */}
      <mesh position={[0, shaftLength + headLength / 2, 0]}>
        <coneGeometry
          args={[thickness * 2, headLength, 32]} // [radius, height, radial segments]
        />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
};

// AxisArrows component
const AxisArrows = ({ length }: { length: number }) => (
  <>
    <Arrow dir={[1, 0, 0]} color={0xff0000} length={length + 1} thickness={0.03} />
    <Arrow dir={[0, 1, 0]} color={0x00ff00} length={length + 1} thickness={0.03} />
    <Arrow dir={[0, 0, 1]} color={0x0000ff} length={length + 1} thickness={0.03} />
  </>
);

// Grid component
const PositiveGridPlanes = () => {
  const gridSize = 10;
  const divisions = 10;

  return (
    <>
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[gridSize / 2, 0, gridSize / 2]} // Positive X and Z axes
      />

      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[gridSize / 2, gridSize / 2, 0]} // Positive X and Y axes
        rotation={[Math.PI / 2, 0, 0]} // Rotate to XY plane
      />

      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[0, gridSize / 2, gridSize / 2]} // Positive Y and Z axes
        rotation={[0, 0, Math.PI / 2]} // Rotate to YZ plane
      />
    </>
  );
};

// Each coordinate label component
const CoordinateLabel = ({ text, position }: { text: string; position: [number, number, number] }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  const { camera } = useThree();

  useEffect(() => {
    const loader = new FontLoader();
    loader.load("/fonts/roboto/Roboto Medium_Regular.json", (font: THREE.Font) => {
      const textGeometry = new TextGeometry(text, {
        font: font,
        size: 0.3,
        height: 0,
      });

      if (meshRef.current) {
        meshRef.current.geometry = textGeometry;
      }
    });
  }, [text]);

  useFrame(() => {
    const handleUpdateRotation = () => {
      if (meshRef.current) {
        meshRef.current.lookAt(camera.position);
      }
    };
    handleUpdateRotation();
  }, [camera]);

  return (
    <mesh ref={meshRef} position={position}>
      <meshBasicMaterial color={0xffffff} />
    </mesh>
  );
};

// Axis labels component with dynamically created labels
const AxisLabels = ({ length }: { length: number }) => {
  const labels = [];
  for (let i = 1; i <= length; i++) {
    labels.push(
      <React.Fragment key={`x-${i}`}>
        <CoordinateLabel text={(i - length / 2).toString()} position={[i, 0, -0.25]} />
      </React.Fragment>,
      <React.Fragment key={`y-${i}`}>
        <CoordinateLabel text={(i - length / 2).toString()} position={[0.25, i, 0]} />
      </React.Fragment>,
      <React.Fragment key={`z-${i}`}>
        <CoordinateLabel text={(i - length / 2).toString()} position={[-0.25, 0, i]} />
      </React.Fragment>,
    );
  }

  return (
    <>
      <CoordinateLabel text={(-length / 2).toString()} position={[0.25, 0.1, 0]} />
      <CoordinateLabel text="X" position={[length + 1, 0, -0.25]} />
      <CoordinateLabel text="Y" position={[0.25, length + 1, 0]} />
      <CoordinateLabel text="Z" position={[-0.25, 0, length + 1]} />
      {labels}
    </>
  );
};

const RobotComponents: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  if (!robot.visible) return null;

  return (
    <object3D key={robot.id}>
      <Nugus model={robot} />
    </object3D>
  );
});

@observer
export class KinematicsView extends React.Component<{
  controller: KinematicsController;
  model: KinematicsModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    return (
      <div className="w-full h-full flex flex-col">
        <Menu>
          <div className="h-full flex items-center justify-end">
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots.map((r) => r.robotModel)}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>

        <div className="flex-1 relative">
          <Canvas camera={{ position: [10, 10, 10], fov: 70 }} className="w-full h-full">
            <CameraControls />

            <ambientLight intensity={0.5} />
            <directionalLight position={[10, 10, 5]} />

            <PositiveGridPlanes />
            <AxisArrows length={10} />
            <AxisLabels length={10} />

            {selectedRobot && <RobotComponents robot={selectedRobot} />}
          </Canvas>
        </div>
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
