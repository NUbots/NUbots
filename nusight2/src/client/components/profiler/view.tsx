import { PropsWithChildren } from "react";
import React from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Icon } from "../icon/view";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { ProfilerController } from "./controller";
import { ProfilerModel, ProfilerRobotModel } from "./model";
import style from "./style.module.css";

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
      controller,
    } = this.props;

    return (
      <div className={style.Profiler}>
        <Menu>
          <div className={style.selector}>
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className="overflow-x-auto p-2">
            <div className="flex-col m-2">
              <SearchBox model={selectedRobot} controller={controller} />
            </div>
            <table className="min-w-full table-auto border-collapse border border-gray-200">
              <thead>
                <tr>
                  <th onClick={() => controller.setSort(selectedRobot!, "reactionId")}>
                    Reaction ID {controller.getSortIcon(selectedRobot!, "reactionId")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "reactor")}>
                    Reactor Name {controller.getSortIcon(selectedRobot!, "reactor")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "name")}>
                    Reaction Name {controller.getSortIcon(selectedRobot!, "name")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "total_time")}>
                    Total Time (ms) {controller.getSortIcon(selectedRobot!, "total_time")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "count")}>
                    Count {controller.getSortIcon(selectedRobot!, "count")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "min_time")}>
                    Min Time (ms) {controller.getSortIcon(selectedRobot!, "min_time")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "max_time")}>
                    Max Time (ms) {controller.getSortIcon(selectedRobot!, "max_time")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "avg_time")}>
                    Average Time (ms) {controller.getSortIcon(selectedRobot!, "avg_time")}
                  </th>
                  <th onClick={() => controller.setSort(selectedRobot!, "percentage")}>
                    Time % {controller.getSortIcon(selectedRobot!, "percentage")}
                  </th>
                </tr>
              </thead>
              <tbody>
                {selectedRobot!.sortedProfiles.map((profile) => (
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
                      <div style={controller.getPercentageStyle(profile.percentage)}>
                        <span>{profile.percentage.toFixed(1)} %</span>
                      </div>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
            <div className="flex-col border border-gray-200 w-full">
              <RobotStats totalCount={selectedRobot.totalCount} totalTime={selectedRobot.totalTime} />
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

interface SearchBoxProps {
  model: ProfilerRobotModel;
  controller: ProfilerController;
}

const SearchBox = observer(function SearchBox(props: SearchBoxProps) {
  const { model, controller } = props;

  return (
    <div className="relative w-64">
      <Icon className="absolute left-1 top-1 text-icon pointer-events-none">search</Icon>
      <input
        type="search"
        className="pl-8 pr-2 h-7 w-[320px] border border-gray-300 rounded bg-white focus:outline-none focus:border-transparent focus:ring-2 focus:ring-nusight-500"
        placeholder="Filter profiles"
        value={model.search}
        onChange={(e) => controller.setSearch(model, e.target.value)}
      />
    </div>
  );
});

export type RobotStatsProps = {
  totalTime: number;
  totalCount: number;
  // TODO: Add device stats, CPU usage, memory usage, etc.
};

export const RobotStats = (props: RobotStatsProps) => {
  return (
    <div className="w-full border rounded-lg">
      <header className={style.header}>Totals</header>
      <div className={style.details}>
        <div className={style.group}>
          <div className={style.row}>
            <span className={style.label}>Total Time (ms)</span>
            {props.totalTime.toFixed(1)}
          </div>
          <div className={style.row}>
            <span className={style.label}>Total Reactions</span>
            {props.totalCount}
          </div>
        </div>
      </div>
    </div>
  );
};
