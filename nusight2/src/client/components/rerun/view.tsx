import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { observer } from "mobx-react";

import { NUsightNetwork } from "../../network/nusight_network";

import { RerunController } from "./controller";
import { RerunModel } from "./model";
import { RerunNetwork } from "./network";
import WebViewer from "@rerun-io/web-viewer-react";

@observer
export class RerunView extends Component<{
  Menu: ComponentType<PropsWithChildren>;
  model: RerunModel;
  network: RerunNetwork;
  controller: RerunController;
}> {
  static of({
    model,
    Menu,
    nusightNetwork,
  }: {
    model: RerunModel;
    Menu: ComponentType<PropsWithChildren>;
    nusightNetwork: NUsightNetwork;
  }): ComponentType {
    const controller = RerunController.of({ model });
    return () => {
      const network = RerunNetwork.of(nusightNetwork, model);
      return <RerunView controller={controller} Menu={Menu} model={model} network={network} />;
    };
  }

  componentWillUnmount(): void {
    this.props.network.destroy();
  }

  render() {
    const { Menu, model, controller } = this.props;
    return (
      <div className="flex flex-col w-full h-full">
        <Menu />
        <div className="flex-grow relative">
          <WebViewer width="100%" height="100%" rrd="https://app.rerun.io/version/0.18.2/examples/dna.rrd" />
        </div>
      </div>
    );
  }
}
