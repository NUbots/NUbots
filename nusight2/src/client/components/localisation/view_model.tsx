import React, { useEffect, useMemo } from "react";
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
import { PerspectiveCamera } from "../three/three_fiber";

import { FieldView } from "./entities/field/view";
import { TextBillboard } from "./entities/text_billboard/view";
import { Robot } from "./entities/robot/view";
import { GridView } from "./entities/grid/view";
import { LocalisationModel } from "./model";
import { LocalisationRobotModel } from "./robot_model";
import { SkyboxView } from "./entities/skybox/view";
import { BoundingBox } from "./entities/bounding_box/view";
import { WalkPathVisualiser } from "./entities/walk_path_visualiser/view";
import { WalkPathGoal } from "./entities/walk_path_goal/view";
import { FieldIntersections } from "./entities/field_intersections/view";

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";


// Ball texture obtained from https://katfetisov.wordpress.com/2014/08/08/freebies-football-textures/
const textureLoader = new THREE.TextureLoader();
const soccerBallTexture = textureLoader.load("/images/ball_texture.png");

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
            {model.walkToDebugVisible &&
                model.robots
                    .filter((robot) => robot.visible && robot.Hfd)
                    .map((robot) => <WalkPathVisualiser key={robot.id} model={robot} />)}
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
            {model.walkToDebugVisible &&
                model.robots
                    .filter((robot) => robot.visible && robot.Hfd)
                    .map((robot) => <WalkPathGoal key={robot.id} model={robot} />)}
            <Robots model={model} />
            {model.boundedBoxVisible &&
                model.robots.map((robot) => {
                    if (robot.visible && robot.boundingBox) {
                        return <BoundingBox key={robot.id} model={robot} />;
                    }
                    return null;
                })}
        </object3D>
    );
});

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
    const label = robotModel.player_id == -1 ? robotModel.purpose : "N" + robotModel.player_id + " " + robotModel.purpose;

    return (
        <TextBillboard
            position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.6]}
            color="robotModel.color"
            backgroundColor="white"

            text={robotModel.purpose}
            cameraPitch={cameraPitch}
            cameraYaw={cameraYaw}
        />
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
