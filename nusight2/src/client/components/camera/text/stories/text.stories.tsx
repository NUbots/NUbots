import * as React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { computed, observable } from "mobx";
import { Observer } from "mobx-react";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { fakeCameraModel } from "../../stories/fake_camera_model";
import { CameraView, Renderable } from "../../view";
import { CameraViewModel } from "../../view_model";
import { TextOpts, TextViewModel } from "../text";

const meta: Meta<typeof TextViewModel> = {
  title: "components/Camera/objects/Text Renderer",
  parameters: {
    layout: "fullscreen",
  },
};

export default meta;

class TextCameraViewModel extends CameraViewModel {
  readonly textData: TextOpts[] = [
    {
      type: "ray",
      text: "Solid green background",
      ray: Vector3.of(0.6876522564815477, -0.598479072268754, 0.4110440052015928),
      textColor: Vector4.of(0, 0, 0, 1),
      backgroundColor: Vector4.of(0, 1, 0, 1),
      height: 24,
    },
    {
      type: "ray",
      text: "Solid blue background, half opacity",
      ray: Vector3.of(0.81835410640818, -0.22666463246164104, -0.5281284890212783),
      textColor: Vector4.of(0, 0, 0, 0.5),
      backgroundColor: Vector4.of(0.058, 0.796, 1, 0.5),
      height: 24,
    },
    {
      type: "ray",
      text: "Red text, no background",
      ray: Vector3.of(0.9330241947595856, 0.21871844860382184, -0.2857063041543961),
      textColor: Vector4.of(1, 0, 0, 1),
      height: 24,
    },
    {
      type: "pixel",
      text: "White text, no background, half opacity",
      pixel: Vector2.of(100, 200),
      textColor: Vector4.of(1, 1, 1, 0.5),
      height: 24,
    },
  ];

  @computed
  private get textViewModel() {
    return new TextViewModel(this.canvas, this.model.params, this.model.image);
  }

  protected getRenderables(): Renderable[] {
    return this.textData.map((text) => this.textViewModel.text(text));
  }
}

export const TextRenderStatic: StoryObj<typeof TextViewModel> = {
  name: "static",
  render: () => {
    const box = observable<{ viewModel: TextCameraViewModel | undefined }>({
      viewModel: undefined,
    });

    fakeCameraModel().then((model) => {
      box.viewModel = new TextCameraViewModel(model);
    });

    return (
      <Observer>
        {() =>
          box.viewModel ? (
            <div className="w-screen h-screen">
              <CameraView viewModel={box.viewModel} viewType="full" objectFit="fill" allowPanAndZoom={true} />
            </div>
          ) : null
        }
      </Observer>
    );
  },
};
