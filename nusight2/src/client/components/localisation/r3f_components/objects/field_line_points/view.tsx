import React from "react";

import { Vector3 } from "../../../../../../shared/math/vector3";

interface FieldLinePointsProps {
    points: Vector3[];
    color?: string;
    size?: number;
}

export const FieldLinePoints: React.FC<FieldLinePointsProps> = ({
    points,
    color = 'blue',
    size = 0.02
}) => {
    return (
        <object3D>
            {points.map((position, index) => (
                <mesh
                    key={index}
                    position={position.add(new Vector3(0, 0, 0.005)).toArray()}
                >
                    <circleBufferGeometry args={[size, 20]} />
                    <meshBasicMaterial color={color} />
                </mesh>
            ))}
        </object3D>
    );
};
