import * as React from "react";
import { useMemo } from "react";
import { Meta, StoryObj } from "@storybook/react";
import { computed, observable } from "mobx";
import { Observer } from "mobx-react";

import { Vector3 } from "../../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../../shared/math/vector4";
import { CameraModel } from "../../../model";
import { fakeCameraModel } from "../../../stories/fake_camera_model";
import { CameraView, Renderable } from "../../../view";
import { CameraViewModel } from "../../../view_model";
import { TextOpts, TextViewModel } from "../text";

type StoryComponent = React.FunctionComponent<{
  text: TextOpts["text"];
  height: TextOpts["height"];
  align: TextOpts["align"];
  baseline: TextOpts["baseline"];
  textColor: string;
  showBackground: boolean;
  backgroundColor: string;
}>;

const meta: Meta<StoryComponent> = {
  title: "components/Camera/objects/Text Renderer",
  parameters: {
    layout: "fullscreen",
  },
  argTypes: {
    text: {
      control: "text",
    },
    height: {
      control: {
        type: "range",
        min: 5,
        max: 80,
        step: 1,
        disableDebounce: true,
      },
    },
    align: {
      control: "inline-radio",
      options: ["start", "middle", "end"],
    },
    baseline: {
      control: "inline-radio",
      options: ["top", "middle", "bottom"],
    },
    textColor: {
      control: "color",
    },
    showBackground: {
      control: "boolean",
    },
    backgroundColor: {
      control: "color",
    },
  },
  args: {
    text: "Insert Text",
    height: 20,
    align: "start",
    baseline: "bottom",
    textColor: "#FFFFFF",
    showBackground: false,
    backgroundColor: "#FFFFFF",
  },
};

export default meta;

function hexToVec4(hexColor: string) {
  const r = parseInt(hexColor.slice(1, 3), 16) / 255;
  const g = parseInt(hexColor.slice(3, 5), 16) / 255;
  const b = parseInt(hexColor.slice(5, 7), 16) / 255;
  return Vector4.of(r, g, b, 1);
}

class TextCameraViewModel extends CameraViewModel {
  @observable accessor textData?: TextOpts;

  @computed
  private get textViewModel() {
    return new TextViewModel(this.canvas, this.model.params, this.model.image);
  }

  protected getRenderables(): Renderable[] {
    return this.textData ? [this.textViewModel.text(this.textData)] : [];
  }
}

const box = observable<{ model: CameraModel | undefined }>({
  model: undefined,
});

fakeCameraModel().then((model) => {
  box.model = model;
});

export const TextRenderInteractive: StoryObj<StoryComponent> = {
  name: "interactive",
  render: ({ text, height, align, baseline, textColor, showBackground, backgroundColor }) => {
    return (
      <Observer>
        {() => {
          const viewModel = useMemo(() => (box.model ? new TextCameraViewModel(box.model) : undefined), [box.model]);

          const textOpts: TextOpts = {
            type: "ray",
            ray: Vector3.of(1, 0, 0),
            text,
            align,
            baseline,
            textColor: hexToVec4(textColor),
            backgroundColor: showBackground ? hexToVec4(backgroundColor) : Vector4.of(0, 0, 0, 0),
            height,
          };

          if (viewModel) {
            viewModel.textData = textOpts;
          }

          return viewModel ? (
            <div className="w-screen h-screen">
              <CameraView viewModel={viewModel} viewType="full" objectFit="fill" allowPanAndZoom={true} />
            </div>
          ) : null;
        }}
      </Observer>
    );
  },
};
