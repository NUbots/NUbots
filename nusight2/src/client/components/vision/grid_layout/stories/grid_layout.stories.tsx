import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { observable } from "mobx";
import { observer } from "mobx-react";

import { SeededRandom } from "../../../../../shared/base/random/seeded_random";
import { range } from "../../../../../shared/base/range";
import useInterval from "../../../../hooks/use-interval";
import sampleImageUrl from "../../../camera/image_view/stories/images/image.jpg";
import { GridLayout } from "../grid_layout";

const imageWidth = 600;
const imageHeight = 480;
const aspectRatio = imageHeight / imageWidth;

type StoryComponent = React.FunctionComponent<{ numItems: number; subItems: number }>;

const meta: Meta<StoryComponent> = {
  title: "components/GridLayout",
  argTypes: {
    numItems: {
      control: { type: "range", min: 1, max: 10 },
    },
    subItems: {
      control: { type: "range", min: 1, max: 10 },
    },
  },
  args: {
    numItems: 2,
    subItems: 1,
  },
  parameters: {
    layout: "fullscreen",
  },
  decorators: [(story) => <div className="flex flex-col w-screen h-screen">{story()}</div>],
};

export default meta;

export const Static: StoryObj<StoryComponent> = {
  name: "static",
  render: ({ numItems, subItems }) => {
    return (
      <>
        <div className="py-2 text-center">Open Storybook Controls to adjust number of items and sub items.</div>
        <div className="h-full">
          <GridLayout itemAspectRatio={aspectRatio}>
            {range(numItems).map((i) => (
              <div key={i} className="relative w-full h-full" style={{ backgroundColor: "red" }}>
                <GridLayout itemAspectRatio={aspectRatio}>
                  {range(subItems).map((i) => (
                    <div key={i} className="relative w-full h-full">
                      <img className="absolute w-full h-full" src={sampleImageUrl} style={{ objectFit: "contain" }} />
                    </div>
                  ))}
                </GridLayout>
              </div>
            ))}
          </GridLayout>
        </div>
      </>
    );
  },
};

export const Animated: StoryObj<StoryComponent> = {
  name: "animated",
  render({ numItems, subItems }) {
    const width = observable.box(1);
    const random = SeededRandom.of("grid_layout");

    const Component = observer(() => {
      useInterval(() => width.set(random.float()), 2000);
      return (
        <>
          <div className="py-2 text-center">Open Storybook Controls to adjust number of items and sub items.</div>
          <div
            style={{
              height: "100%",
              width: `${width.get() * 100}%`,
              transition: "width 500ms ease",
            }}
          >
            <GridLayout itemAspectRatio={aspectRatio}>
              {range(numItems).map((i) => (
                <div key={i} className="relative w-full h-full" style={{ backgroundColor: "red" }}>
                  <GridLayout itemAspectRatio={aspectRatio}>
                    {range(subItems).map((i) => (
                      <div key={i} className="relative w-full h-full">
                        <img className="absolute w-full h-full" src={sampleImageUrl} style={{ objectFit: "contain" }} />
                      </div>
                    ))}
                  </GridLayout>
                </div>
              ))}
            </GridLayout>
          </div>
        </>
      );
    });

    return <Component />;
  },
};
