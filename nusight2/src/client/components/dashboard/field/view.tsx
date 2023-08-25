import React from "react";
import { Component } from "react";
import { observer } from "mobx-react";

import { Renderer } from "../../../render2d/renderer";

import { FieldModel } from "./model";
import style from "./style.module.css";
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
      <div className={style.container}>
        <Renderer
          engine="svg"
          className={style.field}
          scene={viewModel.scene}
          camera={viewModel.camera}
          aspectRatio={viewModel.aspectRatio}
        />
      </div>
    );
  }
}
