import { action } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

import { CameraViewModel } from "./view_model";

export class CameraController {
  private viewModel: CameraViewModel;

  constructor(viewModel: CameraViewModel) {
    this.viewModel = viewModel;
  }

  static of(viewModel: CameraViewModel) {
    return new CameraController(viewModel);
  }

  zoomScroll = (deltaY: number) => {
    const { width, height } = this.viewModel.canvas;

    // Mouse pixel offset from the centre of the canvas
    const lastMouse = this.viewModel.lastMousePosition ?? new Vector2(0, 0);
    const centerOffset = lastMouse.subtract(new Vector2(width / 2, height / 2));

    this.zoom(centerOffset, -Math.sign(deltaY) * 0.25);
  };

  zoomCenter = (zoomAmount: number) => {
    this.zoom(Vector2.of(0, 0), zoomAmount);
  };

  @action
  zoom = (centerOffset: Vector2, zoomAmount: number) => {
    // Pixel offset of the mouse from the centre of the image if the zoom level was 1 (no zoom)
    // Used to zoom in/out while maintaining focus on the mouse position
    const scalelessOffset = this.viewModel.pan.add(centerOffset).divideScalar(this.viewModel.zoom);

    // Set the new pan/zoom
    this.viewModel.zoom = this.limitZoom(this.viewModel.zoom + zoomAmount);

    const newPan = scalelessOffset.multiplyScalar(this.viewModel.zoom).subtract(centerOffset);
    this.viewModel.pan = this.limitPan(newPan);
  };

  @action
  startPan = () => {
    this.viewModel.isPanning = true;
  };

  @action
  endPan = () => {
    this.viewModel.isPanning = false;
  };

  @action
  pan = (x: number, y: number) => {
    const mousePos = new Vector2(x, y);

    // Only pan while left mouse is held
    if (this.viewModel.isPanning && this.viewModel.lastMousePosition !== undefined) {
      const displacement = this.viewModel.lastMousePosition.subtract(mousePos);
      this.viewModel.pan = this.limitPan(this.viewModel.pan.add(displacement));
    }

    // Track the mouse position
    this.viewModel.lastMousePosition = mousePos;
  };

  @action
  resetCamera = () => {
    this.viewModel.zoom = 1;
    this.viewModel.pan = new Vector2(0, 0);
  };

  private limitZoom = (zoom: number) => {
    return Math.min(Math.max(zoom, CameraViewModel.minZoom), CameraViewModel.maxZoom);
  };

  private limitPan = (pan: Vector2) => {
    const zoom = this.viewModel.zoom;
    const widthLimit = (this.viewModel.canvas.width / 2) * (zoom - 1);
    const heightLimit = (this.viewModel.canvas.height / 2) * (zoom - 1);

    // Apply min/max values for pan
    const x = Math.min(widthLimit, Math.max(pan.x, -widthLimit));
    const y = Math.min(heightLimit, Math.max(pan.y, -heightLimit));

    return new Vector2(x, y);
  };
}
