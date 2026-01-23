import React, { ComponentType, PropsWithChildren } from "react";
import { observer } from "mobx-react";

import { GraphView } from "./components/graph_view";
import { DirectorModel } from "./model";

export interface DirectorViewProps {
  model: DirectorModel;
  /**
   * Menu component injected by NUSight (usually robot selector menu bar)
   */
  Menu: ComponentType<PropsWithChildren>;
}

/**
 * View that renders the state of the NUbots Director as an SVG graph.
 * Currently this is a placeholder; the visualisation will be implemented in
 * subsequent iterations once the protobuf example payload is supplied.
 */
export const DirectorView = observer(function DirectorView(props: DirectorViewProps) {
  const { Menu, model } = props;

  return (
    <div className="w-full h-full flex flex-col">
      {/* Top menu bar */}
      <Menu />

      {/* Graph viewport */}
      <div className="flex-grow relative bg-auto-surface-1 overflow-auto p-4">
        {model.graph ? (
          <GraphView graph={model.graph} />
        ) : (
          <div className="w-full h-full flex items-center justify-center">
            <p className="text-lg text-auto-contrast">Waiting for DirectorState...</p>
          </div>
        )}
      </div>
    </div>
  );
});
