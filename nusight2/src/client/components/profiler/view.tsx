import { PropsWithChildren } from "react";
import React from "react";
import { action, observable } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { ProfilerController } from "./controller";
import { ProfilerModel } from "./model";
import { Profile } from "./model";
import styles from "./style.module.css";

@observer
export class ProfilerView extends React.Component<{
  controller: ProfilerController;
  model: ProfilerModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    return (
      <div className={styles.Profiler}>
        <Menu>
          <div className={styles.selector}>
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className="overflow-x-auto">
            <table className="min-w-full table-auto border-collapse border border-gray-200">
              <thead>
                <tr>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "reactionId")}>Reaction ID {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "reactionId")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "reactor")}>Reactor Name {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "reactor")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "name")}>Reaction Name {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "name")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "total_time")}>Total Time (ms) {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "total_time")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "count")}>Count {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "count")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "min_time")}>Min Time (ms) {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "min_time")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "max_time")}>Max Time (ms) {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "max_time")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "avg_time")}>Average Time (ms) {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "avg_time")}</th>
                  <th onClick={() => this.props.controller.setSort(this.props.model.selectedRobot!, "percentage")}>Time % {this.props.controller.getSortIcon(this.props.model.selectedRobot!, "percentage")}</th>
                </tr>
              </thead>
              <tbody>
                {this.props.model.selectedRobot!.sortedProfiles.map((profile) => (
                  <tr key={profile.reactionId}>
                    <td>{profile.reactionId}</td>
                    <td>{profile.reactor}</td>
                    <td>{profile.name.replace(/\w+::/g, "")} </td>
                    <td>{profile.total_time.toFixed(1)}</td>
                    <td>{profile.count}</td>
                    <td>{profile.min_time.toFixed(1)}</td>
                    <td>{profile.max_time.toFixed(1)}</td>
                    <td>{profile.avg_time.toFixed(1)}</td>
                    <td>
                      <div style={this.props.controller.getPercentageStyle(profile.percentage)}>
                        <span>{profile.percentage.toFixed(1)} %</span>
                      </div>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
