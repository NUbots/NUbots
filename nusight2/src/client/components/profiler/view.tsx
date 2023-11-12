import { PropsWithChildren } from "react";
import React from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { ProfilerController } from "./controller";
import { ProfilerModel } from "./model";
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
          <div className={styles.content}>
            <div className={styles.content}>
            <table>
              <thead>
                <tr>
                  <th>Name</th>
                  <th>Total Time (ms)</th>
                  <th>Count</th>
                  <th>Min Time (ms)</th>
                  <th>Max Time (ms)</th>
                  <th>Average Time (ms)</th>
                </tr>
              </thead>
              <tbody>
                {selectedRobot.profiles.map(profile => (
                  <tr key={profile.name}>
                    <td>{profile.name}</td>
                    <td>{profile.total_time}</td>
                    <td>{profile.count}</td>
                    <td>{profile.min_time}</td>
                    <td>{profile.max_time}</td>
                    <td>{profile.avg_time}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
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
