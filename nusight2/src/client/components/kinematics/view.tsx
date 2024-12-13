import React, { PropsWithChildren, useRef, useEffect, useState } from "react";
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
  const spherical = useRef(new THREE.Spherical(20, Math.PI / 2.5, Math.PI / 4)).current;

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
  const shaftLength = length * 0.90;
  const headLength = length * 0.1;

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
    <CoordinateLabel text="X" position={[length + 1, 0, -0.25]} />
    <CoordinateLabel text="Y" position={[0.25, length + 1, 0]} />
    <CoordinateLabel text="Z" position={[-0.25, 0, length + 1]} />
  </>
);

// Grid with labels component
const GridWithLabels = ({ gridSize, divisions }: { gridSize: number; divisions: number }) => {
  const labels = [];

  for (let i = 0; i <= divisions; i++) {
    const value = (i - gridSize / 2) / 10;

    labels.push(
      // X-axis labels
      <React.Fragment key={`x-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[value * 10, -gridSize / 2, (gridSize / 2) + 1]} />
      </React.Fragment>,

      // Y-axis labels
      <React.Fragment key={`y-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[(-gridSize / 2) - 0.5, value * 10, (gridSize / 2) + 0.5]} />
      </React.Fragment>,

      // Z-axis labels
      <React.Fragment key={`z-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[(gridSize / 2) + 0.5, -gridSize / 2, value * 10] } />
      </React.Fragment>,
    );
  }

  return (
    <>
      {/* Grid for Positive X and Z axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[0, -gridSize / 2, 0]}
      />

      {/* Grid for Positive X and Y axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[0, 0, -gridSize / 2]}
        rotation={[Math.PI / 2, 0, 0]}
      />

      {/* Grid for Positive Y and Z axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[-gridSize / 2, 0, 0]}
        rotation={[0, 0, Math.PI / 2]}
      />

      {/* Labels */}
      {labels}
    </>
  );
};

// Each coordinate label component
const CoordinateLabel = ({ text, position }: { text: string; position: [number, number, number] }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  const { camera } = useThree();
  const [color, setColor] = useState<THREE.Color>(new THREE.Color(0xffffff));

  // Observe theme changes using MutationObserver and update the label color
  useEffect(() => {
    const updateColor = () => {
      const computedColor = window.getComputedStyle(document.documentElement).getPropertyValue("color");
      setColor(new THREE.Color(computedColor));
    };

    const observer = new MutationObserver(updateColor);
    observer.observe(document.documentElement, { attributes: true, attributeFilter: ["class"] });

    return () => observer.disconnect();
  }, []);

  // Load the font and create the text geometry
  useEffect(() => {
    const loader = new FontLoader();
    loader.load("/fonts/roboto/Roboto Medium_Regular.json", (font: THREE.Font) => {
      const textGeometry = new TextGeometry(text, {
        font: font,
        size: 0.2,
        height: 0,
      });

      if (meshRef.current) {
        meshRef.current.geometry = textGeometry;
      }
    });
  }, [text]);

  // Update the labels to always face the camera
  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.lookAt(camera.position);
    }
  }, [camera]);

  return (
    <mesh ref={meshRef} position={position}>
      <meshBasicMaterial color={color} />
    </mesh>
  );
};

// NUgus robot components
const RobotComponents: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  if (!robot.visible) return null;

  return (
    <object3D key={robot.id} rotation={[-Math.PI / 2, 0, -Math.PI / 2]} scale={[5, 5, 5]}>
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
          <Canvas camera={{ position: [10, 10, 10], fov: 60 }} className="w-full h-full">
            <CameraControls />

            <ambientLight intensity={0.5} />
            <directionalLight position={[10, 10, 5]} />

            <GridWithLabels gridSize={10} divisions={10} />
            <AxisArrows length={2} />

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
