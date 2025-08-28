import React, { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";

import { Vector3 } from "../../../../shared/math/vector3";

interface FieldSpheres {
    points: Vector3[];
    color?: string;
    size?: number;
}

export const FieldSpheres: React.FC<FieldSpheres> = ({ points, color = "red", size = 0.01 }) => {
    const meshRef = useRef<THREE.InstancedMesh>(null);

    const sphereGeometry = useMemo(() => new THREE.SphereGeometry(size, 16, 12), [size]);
    const sphereMaterial = useMemo(() => new THREE.MeshBasicMaterial({ color }), [color]);

    useEffect(() => {
        if (meshRef.current) {
            const matrix = new THREE.Matrix4();
            points.forEach((point, index) => {
                matrix.setPosition(point.x, point.y, point.z);
                meshRef.current?.setMatrixAt(index, matrix);
            });
            meshRef.current.instanceMatrix.needsUpdate = true;
        }
    }, [points]);

    return <instancedMesh ref={meshRef} args={[sphereGeometry, sphereMaterial, points.length]} />;
};
