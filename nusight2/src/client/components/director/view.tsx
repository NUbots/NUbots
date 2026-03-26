import React, { ComponentType, PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { GraphView } from "./components/graph_view";
import { DirectorController } from "./controller";
import { DirectorModel } from "./model";

export interface DirectorViewProps {
  model: DirectorModel;
  controller: DirectorController;
  Menu: ComponentType<PropsWithChildren>;
}

/**
 * View that renders the state of the NUbots Director as an SVG graph.
 * Currently this is a placeholder; the visualisation will be implemented in
 * subsequent iterations once the protobuf example payload is supplied.
 */
@observer
export class DirectorView extends React.Component<DirectorViewProps> {
  render() {
    const {
      model: { selectedRobot, robots, graph },
      Menu,
    } = this.props;

    return (
      <div className="w-full h-full flex flex-col">
        {/* Top menu bar */}
        <Menu>
          <div className="h-full flex items-center justify-end">
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>

        {/* Graph viewport */}
        <div className="flex-grow relative bg-auto-surface-1 overflow-auto p-4">
          {graph ? (
            <GraphView graph={graph} robotId={selectedRobot?.id} />
          ) : (
            <div className="w-full h-full flex items-center justify-center">
              <p className="text-lg text-auto-contrast">Waiting for DirectorState...</p>
            </div>
          )}
        </div>
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
