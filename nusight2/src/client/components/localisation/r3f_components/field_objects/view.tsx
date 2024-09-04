import React, { useMemo } from "react";
import * as THREE from "three";
import { Vector3 } from "../../../../../shared/math/vector3";

interface FieldObjectProps {
    objects: Array<{
        position: Vector3;
        height?: number;
        radius?: number;
        color?: string;
    }>;
    defaultHeight?: number;
    defaultRadius?: number;
    defaultColor?: string;
}

export const FieldObjects: React.FC<FieldObjectProps> = ({
    objects,
    defaultHeight = 0.8,
    defaultRadius = 0.05,
    defaultColor = "magenta"
}) => {
    const geometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 20), []);
    const material = useMemo(() => new THREE.MeshStandardMaterial(), []);

    return (
        <object3D>
            {objects.map((obj, index) => {
                const height = obj.height ?? defaultHeight;
                const radius = obj.radius ?? defaultRadius;
                const color = obj.color ?? defaultColor;
                const position = obj.position.add(new Vector3(0, 0, height / 2));

                return (
                    <mesh
                        key={index}
                        position={position.toArray()}
                        rotation={[Math.PI / 2, 0, 0]}
                        scale={[radius, height, radius]}
                    >
                        <primitive object={geometry} />
                        <primitive object={material} attach="material" color={color} />
                    </mesh>
                );
            })}
        </object3D>
    );
};
