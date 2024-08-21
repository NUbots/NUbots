import React from "react";
import { LocalisationModel } from "../../model";
import { Vector3 } from "../../../../../shared/math/vector3";

export const FieldIntersections = ({ model }: { model: LocalisationModel }) => {
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
