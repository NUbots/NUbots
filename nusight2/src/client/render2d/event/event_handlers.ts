import { Render2DMouseEvent, Render2DWheelEvent } from "./mouse_event";

/**
 * Set of Input Events for a 2D Renderer
 */
export interface Render2DEventHandlers {
  onMouseDown?: (event: Render2DMouseEvent) => void;
  onMouseUp?: (event: Render2DMouseEvent) => void;
  onMouseEnter?: (event: Render2DMouseEvent) => void;
  onMouseLeave?: (event: Render2DMouseEvent) => void;
  onMouseMove?: (event: Render2DMouseEvent) => void;
  onClick?: (event: Render2DMouseEvent) => void;
  onContextMenu?: (event: Render2DMouseEvent) => void;
  onWheel?: (event: Render2DWheelEvent) => void;
}
