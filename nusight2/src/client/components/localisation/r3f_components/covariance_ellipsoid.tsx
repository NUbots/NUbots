import React, { useMemo } from "react";
import * as THREE from "three";
import { Vector3 } from "../../../../shared/math/vector3";
import { Matrix4 } from "../../../../shared/math/matrix4";

/**
 * Render a translucent σ‑ellipse (really a flat ellipsoid) that visualises the *planar*
 * positional covariance of a robot.
 *
 * The incoming covariance is provided in the project’s serialisation format;
 * we immediately convert it to a **`THREE.Matrix4`** via
 * `THREE.Matrix4().copy(Matrix4.from(cov).toThree())` so that the component can
 * inter‑operate cleanly with the rest of the r3f/three.js scene.
 *
 * Layout is **[x, y, vx, vy]**; the velocity block is ignored.
 */
export type CovarianceEllipsoidProps = {
    /** Centre of the ellipse in field/world coordinates. */
    position: Vector3;
    /** Covariance object – any type accepted, it’s converted with `Matrix4.from` internally. */
    covariance: unknown;
    /** σ multiplier – 2 ≈ 95 % confidence for axis‑aligned Gaussian. */
    scaleFactor?: number;
    color?: string;
    /** Thickness in metres to extrude in *z* (default 0.02 m). */
    zThickness?: number;
};

export const CovarianceEllipsoid: React.FC<CovarianceEllipsoidProps> = ({
    position,
    covariance,
    scaleFactor = 2,
    color = "cyan",
    zThickness = 0.02,
}) => {
    // Convert once per update to a THREE.Matrix4 that shares the scene’s math types.
    const threeCov = useMemo(() => {
        // Matrix4.from handles whatever protobuf / plain‑object comes in.
        return new THREE.Matrix4().copy(Matrix4.from(covariance as any).toThree());
    }, [covariance]);

    const radii = useMemo(() => {
        const e = threeCov.elements;
        const varX = Math.abs(e[0]); // σ²ₓ
        const varY = Math.abs(e[5]); // σ²ᵧ
        return new Vector3(
            scaleFactor * Math.sqrt(varX || 1e-6),
            scaleFactor * Math.sqrt(varY || 1e-6),
            zThickness,
        );
    }, [threeCov, scaleFactor, zThickness]);

    return (
        <mesh
            position={position.toArray() as [number, number, number]}
            scale={radii.toArray() as [number, number, number]}
        >
            {/* unit sphere stretched into a flat ellipsoid */}
            <sphereGeometry args={[1, 24, 24]} />
            <meshStandardMaterial
                color={color}
                transparent
                opacity={0.25}
                depthWrite={false}
            />
        </mesh>
    );
};
