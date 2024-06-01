import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { observer } from "mobx-react";

import { NUsightNetwork } from "../../network/nusight_network";
import { TreeNodeModel } from "../checkbox_tree/model";
import { CheckboxTree } from "../checkbox_tree/view";

import { ChartController } from "./controller";
import { LineChartProps } from "./line_chart/view";
import { ChartModel } from "./model";
import { ChartNetwork } from "./network";
import { TreeLabel } from "./tree_label/view";

@observer
export class ChartView extends Component<{
  Menu: ComponentType<PropsWithChildren>;
  model: ChartModel;
  network: ChartNetwork;
  controller: ChartController;
  LineChart: ComponentType<LineChartProps>;
}> {
  static of({
    model,
    Menu,
    nusightNetwork,
    LineChart,
  }: {
    model: ChartModel;
    Menu: ComponentType<PropsWithChildren>;
    nusightNetwork: NUsightNetwork;
    LineChart: ComponentType<LineChartProps>;
  }): ComponentType {
    const controller = ChartController.of({ model });
    return () => {
      const network = ChartNetwork.of(nusightNetwork, model);
      return <ChartView controller={controller} Menu={Menu} model={model} network={network} LineChart={LineChart} />;
    };
  }

  componentWillUnmount(): void {
    this.props.network.destroy();
  }

  render() {
    const { Menu, model, controller, LineChart } = this.props;
    return (
      <div className="flex flex-col w-full">
        <Menu />
        <div className="flex flex-1">
          <div className="flex-grow flex flex-col">
            <LineChart />
          </div>
          <div className="w-[400px] p-4 border  bg-auto-surface-2">
            <CheckboxTree
              model={model.tree}
              onCheck={controller.onNodeCheck}
              onExpand={controller.onNodeExpand}
              renderLabel={this.renderLabel}
            />
          </div>
        </div>
      </div>
    );
  }

  renderLabel = (node: TreeNodeModel): JSX.Element | string => {
    return (
      <TreeLabel
        node={node}
        onColorChange={this.props.controller.onColorChange}
        onMouseEnter={this.props.controller.onHighlight}
        onMouseLeave={this.props.controller.onUnhighlight}
      />
    );
  };
}
