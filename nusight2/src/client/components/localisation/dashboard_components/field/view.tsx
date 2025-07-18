import React from "react";
import { Component } from "react";
import { observer } from "mobx-react";

import { Renderer } from "../../../../render2d/renderer";

import { DashboardFieldModel } from "./model";
import { DashboardFieldViewModel } from "./view_model";

export type FieldProps = {
  model: DashboardFieldModel;
};

@observer
export class DashboardFieldView extends Component<FieldProps> {
  render() {
    const model = this.props.model;
    const viewModel = DashboardFieldViewModel.of(model);
    return (
      <div className="h-full m-0 p-0 bottom-0 absolute w-full">
        <Renderer
          engine="svg"
          className="h-full m-0 p-0 absolute w-full"
          scene={viewModel.scene}
          camera={viewModel.camera}
          aspectRatio={viewModel.aspectRatio}
        />
      </div>
    );
  }
}
