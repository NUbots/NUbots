import React, { useRef } from "react";
import { useEffect } from "react";

/** Scrolling using the mouse wheel from inside this element is prevented */
export function PreventMouseScroll(props: { className?: string; children?: React.ReactNode }) {
  const element = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const preventScroll = (e: WheelEvent) => e.preventDefault();
    element.current?.addEventListener("wheel", preventScroll, { passive: false });
    return () => {
      element.current?.removeEventListener("wheel", preventScroll);
    };
  }, []);

  return (
    <div ref={element} className={props.className}>
      {props.children}
    </div>
  );
}
