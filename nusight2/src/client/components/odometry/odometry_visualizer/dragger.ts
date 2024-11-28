import React, { useRef } from "react";

import { Vector2 } from "../../../../shared/math/vector2";

import { OdometryCamera, OdometryVisualizerModel } from "./model";

type Drag = {
  fromCamera: OdometryCamera;
  fromMouse: Vector2;
};

export const useDragger = (model: OdometryVisualizerModel) => {
  const drag = useRef<Drag | undefined>(undefined);

  const onWheel = React.useCallback(
    (event: React.WheelEvent) => {
      const { camera } = model;
      camera.distance = clamp(camera.distance * Math.pow(-1.95, event.deltaY / 20), 0.01, 10, 0);
    },
    [model],
  );

  const onMouseDown = React.useCallback(
    (event: React.MouseEvent) => {
      drag.current = {
        fromCamera: model.camera.clone(),
        fromMouse: Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY),
      };
    },
    [model],
  );

  const onMouseMove = React.useCallback((event: React.MouseEvent) => {
    if (!drag.current) {
      return;
    }
    update(drag.current, Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY));
  }, []);

  const onMouseUp = React.useCallback((event: React.MouseEvent) => {
    if (!drag.current) {
      return;
    }
    update(drag.current, Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY));
    drag.current = undefined;
  }, []);

  function update(drag: Drag, to: Vector2) {
    const delta = to.subtract(drag.fromMouse);
    const scale = 1 / 100;
    model.camera.pitch = clamp(drag.fromCamera.pitch - delta.y / 100, -Math.PI / 2, Math.PI / 2);
    model.camera.yaw = drag.fromCamera.yaw + scale * delta.x;
  }

  return {
    onWheel,
    onMouseDown,
    onMouseMove,
    onMouseUp,
  };
};

const clamp = (x: number, min: number, max: number, eps = 1e-9): number => {
  return Math.max(min + eps, Math.min(max - eps, x));
};
