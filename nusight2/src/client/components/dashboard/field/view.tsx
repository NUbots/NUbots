import React from "react";
import { Component } from "react";
import { observer } from "mobx-react";

import { Renderer } from "../../../render2d/renderer";

import { FieldModel } from "./model";
import { FieldViewModel } from "./view_model";

export type FieldProps = {
  model: FieldModel;
};

@observer
export class Field extends Component<FieldProps> {
  render() {
    const model = this.props.model;
    const viewModel = FieldViewModel.of(model);
    return (
      <div className="h-full m-0 p-0 bottom-0 absolute w-full">
        <Renderer
          engine="svg"
          className="h-full m-0 p-0 bg-orangegray-200 absolute w-full"
          scene={viewModel.scene}
          camera={viewModel.camera}
          aspectRatio={viewModel.aspectRatio}
        />
      </div>
    );
  }
}
