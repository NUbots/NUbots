import React from "react";
import { autorun, computed } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import IconRobot from "../robot_selector/icon_robot";
import { Option, Select } from "../select/view";

import IconPlug from "./icon_plug";

export type RobotSelectorSingleProps = {
  autoSelect?: boolean;
  robots: RobotModel[];
  selected?: RobotModel;
  dropDirection?: "up" | "down";
  onSelect(robot?: RobotModel): void;
};

@observer
export class RobotSelectorSingle extends React.Component<RobotSelectorSingleProps> {
  private disposeAutoSelect?: () => void;

  componentDidMount() {
    if (this.props.autoSelect && !this.disposeAutoSelect) {
      this.disposeAutoSelect = autorun(() => {
        // Automatically select the first option if we don't have a selection
        // and there are options
        if (!this.props.selected && this.props.robots.length > 0) {
          this.props.onSelect(this.props.robots[0]);

          // Clean up after the first run, since we auto select only once
          // during the lifetime of the component
          if (this.disposeAutoSelect) {
            this.disposeAutoSelect();
            this.disposeAutoSelect = undefined;
          }
        }
      });
    }
  }

  componentWillUnmount() {
    if (this.disposeAutoSelect) {
      this.disposeAutoSelect();
      this.disposeAutoSelect = undefined;
    }
  }

  render() {
    const { dropDirection } = this.props;
    return (
      <div>
        <Select
          options={this.options}
          selectedOption={this.selectedOption}
          onChange={this.onChange}
          placeholder="Select a robot..."
          empty={this.renderEmpty}
          icon={<IconRobot className="h-[20px] w-[20px]" />}
          dropDirection={dropDirection}
        />
      </div>
    );
  }

  @computed
  private get renderEmpty() {
    return (
      <div className={" p-[1em] text-center"}>
        <IconPlug className="bg-gray-500 dark:bg-gray-600 fill-gray-200 h-[3em] w-[3em] rounded-full mx-auto mt-[1em] p-[0.8em]" />
        <div className={"text-md py-2 whitespace-nowrap"}>No connected robots</div>
        <span className={"text-sm whitespace-nowrap"}>Run yarn start:sim to simulate robots</span>
      </div>
    );
  }

  @computed
  private get options() {
    return this.props.robots.map((robot) => ({
      id: robot.id,
      label: robot.name,
      robot,
    }));
  }

  @computed
  private get selectedOption() {
    if (this.props.selected) {
      return {
        id: this.props.selected.id,
        label: this.props.selected.name,
      };
    }
  }

  private onChange = (option: Option) => {
    this.props.onSelect(this.props.robots.find((robot) => robot.id === option.id)!);
  };
}
