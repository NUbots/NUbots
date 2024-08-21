import React, { useMemo } from 'react';
import * as THREE from 'three';

// Ball texture obtained from https://katfetisov.wordpress.com/2014/08/08/freebies-football-textures/
const textureLoader = new THREE.TextureLoader();
const soccerBallTexture = textureLoader.load("/images/ball_texture.png");

interface BallProps {
    position: [number, number, number];
    scale: number;
}

export const Ball: React.FC<BallProps> = ({ position, scale }) => {
    const ballGeometry = useMemo(() => new THREE.SphereGeometry(1, 32, 32), []);
    const ballMaterial = useMemo(() => new THREE.MeshStandardMaterial({ map: soccerBallTexture }), []);

    return (
        <mesh
            position={position}
            scale={[scale, scale, scale]}
            geometry={ballGeometry}
            material={ballMaterial}
        />
    );
};

export default Ball;
