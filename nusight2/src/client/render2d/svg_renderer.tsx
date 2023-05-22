import React, { useMemo, useRef, useState } from "react";
import classNames from "classnames";
import ReactResizeDetector from "react-resize-detector";

import { Transform } from "../../shared/math/transform";
import { Vector2 } from "../../shared/math/vector2";

import { RendererProps } from "./renderer";
import style from "./style.module.css";
import { Group } from "./svg/group";
import { toSvgEventHandlers, toSvgTransform } from "./svg/rendering";

export interface SVGRendererTransforms {
  camera: Transform;
  world: Transform;
  getSVGOffset: () => Vector2;
}

export const rendererTransformsContext = React.createContext<SVGRendererTransforms>({
  camera: Transform.of(),
  world: Transform.of(),
  getSVGOffset: () => Vector2.of(),
});

export const SVGRenderer = (props: RendererProps) => {
  const { className, scene, camera, aspectRatio, eventHandlers = {} } = props;
  const [resolution, setResolution] = useState(Transform.of());
  const svgElement = useRef<SVGSVGElement | null>(null);

  const onResize = (width: number, height: number) => {
    setResolution(calculateResolution(width, height, aspectRatio));
    props.onResize?.(width, height);
  };

  const transforms = useMemo(() => {
    const getSVGOffset = () => {
      if (!svgElement.current) {
        return Vector2.of(0, 0);
      }

      const bounds = svgElement.current.getBoundingClientRect();
      return Vector2.of(bounds.x, bounds.y);
    };

    const world = resolution.then(camera.inverse());

    return {
      world,
      svg: { camera: resolution, world, getSVGOffset },
      scene: { camera: world, world, getSVGOffset },
    };
  }, [resolution, camera]);

  return (
    <div className={classNames(className, style.container)}>
      <ReactResizeDetector handleWidth handleHeight onResize={onResize} />
      <svg ref={svgElement} className={style.container} {...toSvgEventHandlers(eventHandlers, transforms.svg)}>
        <g transform={toSvgTransform(transforms.world)}>
          <rendererTransformsContext.Provider value={transforms.scene}>
            <Group model={scene} />
          </rendererTransformsContext.Provider>
        </g>
      </svg>
    </div>
  );
};

function calculateResolution(width: number, height: number, aspectRatio?: number) {
  // Translate to the center
  const translate = Vector2.of(width * 0.5, height * 0.5);

  // If we have an aspect ratio, use it to scale the canvas to unit size
  if (aspectRatio !== undefined) {
    const canvasAspect = width / height;
    const scale = canvasAspect < aspectRatio ? width : height * aspectRatio;
    // Scale to fit
    return Transform.of({ scale: { x: scale, y: -scale }, translate });
  } else {
    return Transform.of({ scale: { x: width, y: -height }, translate });
  }
}
