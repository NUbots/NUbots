import React, { useCallback, useMemo, useRef, useState } from "react";
import classNames from "classnames";

import { useResize } from "../../hooks/use_resize";

import { ResizeContainerModel } from "./model";
import { ResizePanelProps, ResizePanelState } from "./resize_panel";

type OneOrMore<T> = T | T[];

/** Context to provide functions to the child panels */
interface ResizeContainerContext {
  setPanelState: (index: number, state: ResizePanelState) => void;
}

export const resizeContainerContext = React.createContext<ResizeContainerContext>({ setPanelState: () => {} });

/** The context to provide to each resize panel */
interface ResizePanelContext {
  index: number;
  size: number;
  state: ResizePanelState;
}

export const resizePanelContext = React.createContext<ResizePanelContext>({
  index: -1,
  size: 0,
  state: "default",
});

interface ResizeContainerProps {
  /**
   * Key used for saving the sizes of this container's children. This allows the container
   * to maintain its children's sizes across page reloads.
   */
  saveKey?: string;
  /** If included, the contents of the container will be placed horizontally */
  horizontal?: boolean;
  /** Class name for the element of the container */
  className?: string;
  children: OneOrMore<React.ReactElement<ResizePanelProps>>;
}

export function ResizeContainer({ saveKey, horizontal, className, children }: ResizeContainerProps) {
  const childCount = React.Children.count(children);
  const model = useMemo(
    () =>
      new ResizeContainerModel(
        React.Children.map(children, (child) => child.props),
        saveKey,
      ),
    [childCount, saveKey],
  );

  // Index of the handle currently being moved. -1 indicates none
  const dragHandleIndex = useRef(-1);
  const [dragHandleAtLimit, setDragHandleAtLimit] = useState(false);

  // Ref for the container for reacting to size changes
  const containerRef = useRef<HTMLDivElement>(null);
  const containerSize = useRef(0);

  // The sizes that the children are being rendered at
  const [renderedRatios, setRenderedRatios] = useState<number[]>(() => [...model.currentRatios]);

  // Position of the handle at a given index. This is the sum of the rendered child sizes
  // from index 0 to the given index.
  function getHandlePositionOf(index: number) {
    return renderedRatios.slice(0, index + 1).reduce((prev, curr) => (prev += curr), 0);
  }

  // When the container resizes we track the size and adjust the children's sizes to make sure
  // they don't go below their minimum.
  useResize(containerRef, (container) => {
    // Exit if dragging a handle since drag updates trigger this function
    if (dragHandleIndex.current > -1) {
      return;
    }

    containerSize.current = horizontal ? container.clientWidth : container.clientHeight;
    const newRatios = model.getDisplayRatios(containerSize.current);

    // Check if any have changed before updating to prevent infinite updates
    if (renderedRatios.some((size, i) => size !== newRatios[i])) {
      setRenderedRatios(newRatios);
    }
  });

  // When a handle is dragged, update the children's sizes to expand the target child
  function onHandleDrag(e: React.PointerEvent) {
    const container = containerRef.current;
    if (dragHandleIndex.current === -1 || !container) {
      return renderedRatios;
    }

    // Get the mouse position
    const containerDims = container.getBoundingClientRect();
    const mousePos = horizontal ? e.clientX - containerDims.left : e.clientY - containerDims.top;

    // Reset all the states to default so minimized/maximized elements don't prevent resizing
    model.childStates.fill("default");

    const { newRatios, atLimit } = model.moveHandle(dragHandleIndex.current, containerSize.current, mousePos);

    // Convert the ratios to pixels for rendering
    if (renderedRatios.some((size, i) => size !== newRatios[i])) {
      setRenderedRatios(newRatios);
    }

    if (atLimit !== dragHandleAtLimit) {
      setDragHandleAtLimit(atLimit);
    }

    return newRatios;
  }

  const setPanelState = useCallback(
    (index: number, state: ResizePanelState) => {
      model.setPanelState(index, state);
      setRenderedRatios(model.getDisplayRatios(containerSize.current));
    },
    [model],
  );

  const containerContextValue = useMemo(() => ({ setPanelState }), [setPanelState]);

  return (
    <resizeContainerContext.Provider value={containerContextValue}>
      <div
        ref={containerRef}
        className="relative w-full h-full overflow-hidden"
        style={{
          minHeight: horizontal ? undefined : `${model.minSize}px`,
          minWidth: horizontal ? `${model.minSize}px` : undefined,
          maxHeight: horizontal ? undefined : `${model.maxSize}px`,
          maxWidth: horizontal ? `${model.maxSize}px` : undefined,
        }}
      >
        <div className={classNames("flex w-full h-full", { "flex-col": !horizontal }, className)}>
          {/* The content */}
          {React.Children.map(children, (child, i) => (
            <resizePanelContext.Provider
              value={{ index: i, size: renderedRatios[i] * 100, state: model.childStates[i] }}
            >
              {child}
            </resizePanelContext.Provider>
          ))}

          {/* The handles for adjusting sizes */}
          {Array.from(Array(childCount - 1).keys()).map((i) => (
            <ResizeHandle
              key={i}
              horizontal={horizontal}
              atLimit={dragHandleAtLimit}
              position={getHandlePositionOf(i)}
              onStartDrag={() => {
                model.setRatios(renderedRatios);
                dragHandleIndex.current = i;
              }}
              onDrag={onHandleDrag}
              onEndDrag={(e) => {
                model.setRatios(onHandleDrag(e));
                dragHandleIndex.current = -1;
                setDragHandleAtLimit(false);
              }}
            />
          ))}
        </div>
      </div>
    </resizeContainerContext.Provider>
  );
}

interface ResizeHandleProps {
  position: number;
  horizontal?: boolean;
  atLimit?: boolean;
  onStartDrag: () => void;
  onDrag: (event: React.PointerEvent) => void;
  onEndDrag: (event: React.PointerEvent) => void;
}

function ResizeHandle({ position, horizontal, atLimit, onStartDrag, onDrag, onEndDrag }: ResizeHandleProps) {
  // Create a ref for setting pointer capture during drag
  const ref = useRef<HTMLDivElement>(null);
  return (
    <div
      ref={ref}
      className={classNames("group absolute flex justify-center items-center", {
        "w-3 h-full cursor-ew-resize -translate-x-1/2": horizontal,
        "w-full h-3 cursor-ns-resize -translate-y-1/2": !horizontal,
      })}
      onPointerDown={(e) => {
        if (e.button === 0) {
          ref.current?.setPointerCapture(e.pointerId);
          onStartDrag();
        }
      }}
      onPointerUp={(e) => {
        onDrag(e);
        onEndDrag(e);
        ref.current?.releasePointerCapture(e.pointerId);
      }}
      onPointerMove={onDrag}
      style={{
        top: horizontal ? undefined : `${position * 100}%`,
        left: horizontal ? `${position * 100}%` : undefined,
      }}
    >
      <div
        className={classNames(
          "transition-[width,height,background-color] duration-200 bg-auto-contrast-2",
          atLimit ? "group-hover:bg-red-500" : "group-hover:bg-nusight-400 group-hover:dark:bg-nusight-500",
          {
            "w-px h-full mx-auto group-hover:w-[3px]": horizontal,
            "h-px w-full my-auto group-hover:h-[3px]": !horizontal,
          },
        )}
      ></div>
    </div>
  );
}
