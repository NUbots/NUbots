import React from "react";

export function fullscreen(story: () => JSX.Element) {
  return <div className="w-screen h-screen">{story()}</div>;
}
