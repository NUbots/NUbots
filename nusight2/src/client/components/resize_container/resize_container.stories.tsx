import React, { useRef, useState } from "react";
import type { Meta, StoryObj } from "@storybook/react";

import { IconButton } from "../icon_button/view";

import { ResizeContainer } from "./resize_container";
import { ResizePanel, ResizePanelRef, ResizePanelState } from "./resize_panel";

const meta: Meta<typeof ResizeContainer> = {
  title: "components/ResizeContainer",
  component: ResizeContainer,
  parameters: {
    layout: "fullscreen",
  },
};

export default meta;

type Story = StoryObj<typeof ResizeContainer>;

export const Vertical: Story = {
  render: () => {
    const panelRef = useRef<ResizePanelRef>(null);
    const [panelState, setPanelState] = useState<ResizePanelState>("default");

    return (
      <div className="w-screen h-screen">
        <ResizeContainer>
          <ResizePanel minSize={48} className="flex bg-amber-400 p-2 gap-2 flex-col items-center justify-center">
            <span className="w-28 p-1 text-center">Section 1</span>
          </ResizePanel>
          <ResizePanel
            ref={panelRef}
            minSize={92}
            onStateChange={setPanelState}
            className="flex bg-red-400 p-2 gap-2 flex-col items-center justify-center"
          >
            <span className="w-28 p-1 text-center">Section 2</span>
            <div className="flex gap-2 justify-center">
              <IconButton onClick={() => panelRef.current?.setState("minimized")} disabled={panelState === "minimized"}>
                unfold_less
              </IconButton>
              <IconButton onClick={() => panelRef.current?.setState("maximized")} disabled={panelState === "maximized"}>
                unfold_more
              </IconButton>
              <IconButton onClick={() => panelRef.current?.setState("default")} disabled={panelState === "default"}>
                undo
              </IconButton>
            </div>
          </ResizePanel>
          <ResizePanel minSize={48} className="flex bg-teal-400 p-2 gap-2 flex-col items-center justify-center">
            <span className="w-28 p-1 text-center">Section 3</span>
          </ResizePanel>
          <ResizePanel minSize={48} className="flex bg-lime-400 p-2 gap-2 flex-col items-center justify-center">
            <span className="w-28 p-1 text-center">Section 4</span>
          </ResizePanel>
          <ResizePanel minSize={48} className="flex bg-pink-400 p-2 gap-2 flex-col items-center justify-center">
            <span className="w-28 p-1 text-center">Section 5</span>
          </ResizePanel>
        </ResizeContainer>
      </div>
    );
  },
};

export const Horizontal: Story = {
  render: () => {
    return (
      <div className="w-screen h-screen">
        <ResizeContainer horizontal>
          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 1</span>
            <div className="h-32 w-full bg-amber-400 border border-gray-400"></div>
          </ResizePanel>
          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 2</span>
            <div className="h-32 w-full bg-red-400 border border-gray-400"></div>
          </ResizePanel>
          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 3</span>
            <div className="h-32 w-full bg-teal-400 border border-gray-400"></div>
          </ResizePanel>
          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 4</span>
            <div className="h-32 w-full bg-lime-400 border border-gray-400"></div>
          </ResizePanel>
          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 5</span>
            <div className="h-32 w-full bg-pink-400 border border-gray-400"></div>
          </ResizePanel>
        </ResizeContainer>
      </div>
    );
  },
};

export const Nested: Story = {
  render: () => {
    return (
      <div className="w-screen h-screen">
        <ResizeContainer>
          <ResizePanel minSize={100}>
            <ResizeContainer horizontal>
              <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                <span className="shrink-0">Section 1.1</span>
                <div className="w-full h-full bg-red-400 border border-gray-400"></div>
              </ResizePanel>
              <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                <span className="shrink-0">Section 1.2</span>
                <div className="w-full h-full bg-cyan-400 border border-gray-400"></div>
              </ResizePanel>
              <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                <span className="shrink-0">Section 1.3</span>
                <div className="w-full h-full bg-purple-400 border border-gray-400"></div>
              </ResizePanel>
            </ResizeContainer>
          </ResizePanel>

          <ResizePanel minSize={200}>
            <ResizeContainer horizontal>
              <ResizePanel minSize={200}>
                <ResizeContainer>
                  <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                    <span className="shrink-0">Section 2.1.1</span>
                    <div className="w-full h-full bg-amber-400 border border-gray-400"></div>
                  </ResizePanel>
                  <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                    <span className="shrink-0">Section 2.1.2</span>
                    <div className="w-full h-full bg-amber-400 border border-gray-400"></div>
                  </ResizePanel>
                </ResizeContainer>
              </ResizePanel>
              <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                <span className="shrink-0">Section 2.2</span>
                <div className="w-full h-full bg-pink-400 border border-gray-400"></div>
              </ResizePanel>
              <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
                <span className="shrink-0">Section 2.3</span>
                <div className="w-full h-full bg-lime-400 border border-gray-400"></div>
              </ResizePanel>
            </ResizeContainer>
          </ResizePanel>

          <ResizePanel minSize={100} className="flex p-2 gap-2 flex-col">
            <span className="shrink-0">Section 3</span>
            <div className="w-full h-full bg-teal-400 border border-gray-400"></div>
          </ResizePanel>
        </ResizeContainer>
      </div>
    );
  },
};
