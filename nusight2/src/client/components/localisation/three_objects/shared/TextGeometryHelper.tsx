import roboto from "../../../../../assets/fonts/roboto/Roboto Medium_Regular.json";

import { TextGeometry } from 'three/examples/jsm/geometries/TextGeometry';
import { FontLoader } from 'three/examples/jsm/loaders/FontLoader';

export const TextGeometryHelper = (x: string) => {
    const font = new FontLoader().parse(roboto);
    return new TextGeometry(x, {
        font: font,
        size: 0.1,
        height: 0,
    }).center();
};
