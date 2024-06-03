import { PropsWithChildren } from "react";
import React from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Icon } from "../icon/view";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { ProfilerController } from "./controller";
import { ProfilerModel, ProfilerRobotModel } from "./model";

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
      <div className={"flex flex-grow flex-shrink flex-col text-center relative w-full h-full"}>
        <Menu>
          <div className={"h-full flex items-center justify-end"}>
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className="overflow-x-auto p-8">
            <div>
              <RobotStats totalCount={selectedRobot.totalCount} totalTime={selectedRobot.totalTime} />
            </div>
            <div className="flex-col py-4">
              <SearchBox model={selectedRobot} controller={controller} />
            </div>

            <table className="min-w-full table-auto bg-auto-surface-1 border border-collapse border-gray-300 dark:border-gray-700">
              <thead>
                <tr>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "reactionId")}>
                    Reaction ID {controller.getSortIcon(selectedRobot!, "reactionId")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "reactor")}>
                    Reactor Name {controller.getSortIcon(selectedRobot!, "reactor")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "name")}>
                    Reaction Name {controller.getSortIcon(selectedRobot!, "name")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "total_time")}>
                    Total Time (ms) {controller.getSortIcon(selectedRobot!, "total_time")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "count")}>
                    Count {controller.getSortIcon(selectedRobot!, "count")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "min_time")}>
                    Min Time (ms) {controller.getSortIcon(selectedRobot!, "min_time")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "max_time")}>
                    Max Time (ms) {controller.getSortIcon(selectedRobot!, "max_time")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "avg_time")}>
                    Average Time (ms) {controller.getSortIcon(selectedRobot!, "avg_time")}
                  </th>
                  <th className="cursor-pointer select-none" onClick={() => controller.setSort(selectedRobot!, "percentage")}>
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
                        <span className="px-2">{profile.percentage.toFixed(1)} %</span>
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

interface SearchBoxProps {
  model: ProfilerRobotModel;
  controller: ProfilerController;
}

const SearchBox = observer(function SearchBox(props: SearchBoxProps) {
  const { model, controller } = props;

  return (
    <div className="relative w-64">
      <Icon className="absolute left-1 top-1 pointer-events-none">search</Icon>
      <input
        type="search"
        className="pl-8 pr-2 h-7 w-[320px] bg-auto-surface-1 border border-auto rounded-md rounded focus:outline-none focus:border-transparent focus:ring-2 ring-auto"
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
};

export const RobotStats = (props: RobotStatsProps) => {
  return (
    <div>
      <div>
        <div className="flex justify-center pt-8">
          <div className="px-12 py-4 bg-auto-surface-1 border border-auto rounded-md mx-12">
            <div className="text-lg text-auto-secondary">Total Time (s)</div>
            <div className="text-3xl">{(props.totalTime / 1e3).toFixed(1)}</div>
          </div>

          <div className="px-12 py-4 bg-auto-surface-1 border border-auto rounded-md ">
            <div className="text-lg text-auto-secondary">Total Reactions</div>
            <div className="text-3xl">{props.totalCount.toLocaleString()}</div>
          </div>
        </div>
      </div>
    </div>
  );
};
