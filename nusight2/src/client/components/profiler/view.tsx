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
  @observable sortColumn = "name";
  @observable sortOrder = "asc"; // 'asc' or 'desc'

  @action.bound
  setSort(column: string) {
    if (this.sortColumn === column) {
      this.sortOrder = this.sortOrder === "asc" ? "desc" : "asc";
    } else {
      this.sortColumn = column;
      this.sortOrder = "asc";
    }
  }

  sortProfiles(profiles: Profile[]) {
    return profiles.slice().sort((a, b) => {
      if (a[this.sortColumn] < b[this.sortColumn]) {
        return this.sortOrder === "asc" ? -1 : 1;
      }
      if (a[this.sortColumn] > b[this.sortColumn]) {
        return this.sortOrder === "asc" ? 1 : -1;
      }
      return 0;
    });
  }

  getSortIcon(column: string) {
    if (this.sortColumn === column) {
      return this.sortOrder === "asc" ? "↑" : "↓";
    }
    return "";
  }

  getPercentageStyle(percentage: number) {
    // Clamp the percentage between 0 and 100
    const clampedPercentage = Math.min(Math.max(percentage, 0), 100);
    return {
      width: `${clampedPercentage}%`,
      backgroundColor: "lightblue",
      height: "100%",
    };
  }

  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    const sortedProfiles = selectedRobot ? this.sortProfiles(selectedRobot.profiles) : [];

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
                  <th onClick={() => this.setSort("reactionId")}>Reaction ID {this.getSortIcon("reactionId")}</th>
                  <th onClick={() => this.setSort("reactor")}>Reactor Name {this.getSortIcon("reactor")}</th>
                  <th onClick={() => this.setSort("name")}>Reaction Name {this.getSortIcon("name")}</th>
                  <th onClick={() => this.setSort("total_time")}>Total Time (ms) {this.getSortIcon("total_time")}</th>
                  <th onClick={() => this.setSort("count")}>Count {this.getSortIcon("count")}</th>
                  <th onClick={() => this.setSort("min_time")}>Min Time (ms) {this.getSortIcon("min_time")}</th>
                  <th onClick={() => this.setSort("max_time")}>Max Time (ms) {this.getSortIcon("max_time")}</th>
                  <th onClick={() => this.setSort("avg_time")}>Average Time (ms) {this.getSortIcon("avg_time")}</th>
                  <th onClick={() => this.setSort("percentage")}>Time % {this.getSortIcon("percentage")}</th>
                </tr>
              </thead>
              <tbody>
                {sortedProfiles.map((profile) => (
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
                      <div style={this.getPercentageStyle(profile.percentage)}>
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
