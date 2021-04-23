import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'
import { ComponentType } from 'react'

import { NUsightNetwork } from '../../network/nusight_network'
import { TreeNodeModel } from '../checkbox_tree/model'
import { CheckboxTree } from '../checkbox_tree/view'

import { ChartController } from './controller'
import { LineChartProps } from './line_chart/view'
import { ChartModel } from './model'
import { ChartNetwork } from './network'
import style from './style.css'
import { TreeLabel } from './tree_label/view'

@observer
export class ChartView extends Component<{
  Menu: ComponentType
  model: ChartModel
  network: ChartNetwork
  controller: ChartController
  LineChart: ComponentType<LineChartProps>
}> {
  static of({
    model,
    menu,
    nusightNetwork,
    LineChart,
  }: {
    model: ChartModel
    menu: ComponentType
    nusightNetwork: NUsightNetwork
    LineChart: ComponentType<LineChartProps>
  }): ComponentType {
    const controller = ChartController.of({ model })
    return () => {
      const network = ChartNetwork.of(nusightNetwork, model)
      return (
        <ChartView
          controller={controller}
          Menu={menu}
          model={model}
          network={network}
          LineChart={LineChart}
        />
      )
    }
  }

  componentWillUnmount(): void {
    this.props.network.destroy()
  }

  render() {
    const { Menu, model, controller, LineChart } = this.props
    return (
      <div className={style.page}>
        <Menu>
          <ul className={style.menu}>
            <li className={style.menuItem}>
              <button className={style.menuButton}>Line Chart</button>
              <button className={style.menuButton}>2D Scatter</button>
            </li>
          </ul>
        </Menu>
        <div className={style.chart}>
          <div className={style.main}>
            <LineChart />
          </div>
          <div className={style.sidebar}>
            <CheckboxTree
              model={model.tree}
              onCheck={controller.onNodeCheck}
              onExpand={controller.onNodeExpand}
              renderLabel={this.renderLabel}
            />
          </div>
        </div>
      </div>
    )
  }

  renderLabel = (node: TreeNodeModel): JSX.Element | string => {
    return (
      <TreeLabel
        node={node}
        onColorChange={this.props.controller.onColorChange}
        onMouseEnter={this.props.controller.onHighlight}
        onMouseLeave={this.props.controller.onUnhighlight}
      />
    )
  }
}
