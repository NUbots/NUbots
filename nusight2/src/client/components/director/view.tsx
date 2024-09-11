import React from "react";
import { PropsWithChildren } from "react";
import { observer } from "mobx-react";

import { RobotSelectorSingle } from "../robot_selector_single/view";

import { DirectorController } from "./controller";
import { DirectorModel } from "./model";
import { DirectorNetwork } from "./network";

interface DirectorViewProps {
  controller: DirectorController;
  model: DirectorModel;
  network: DirectorNetwork;
  Menu: React.ComponentType<PropsWithChildren<any>>;
}

export const DirectorView = observer(function DirectorView(props: DirectorViewProps) {
  const { Menu, model, controller } = props;
  const { robots, selectedDirectorRobot } = model;

  return (
    <div className="w-full flex flex-col">
      <Menu>
        <div className="h-full flex items-center justify-end">
          <RobotSelectorSingle
            autoSelect
            robots={robots}
            selected={selectedDirectorRobot?.robotModel}
            onSelect={controller.onSelectRobot}
          />
        </div>
      </Menu>
      <div className="m-2">
        <div className="m-4 bg-auto-surface-2 p-4 border border-2 border-auto rounded">
          <div className="text-lg bg-auto-surface-1 py-1 px-4 w-fit h-fit rounded mb-4">Root Tasks</div>
          <div className="bg-auto-surface-1 p-2 rounded flex flex-wrap">
            {Array.from(selectedDirectorRobot?.rootTasks || []).map((rootTask) => (
              <div
                key={rootTask}
                className="bg-auto-surface-1 flex min-w-[7em] w-fit h-fit p-4 m-1 rounded border border-2 border-auto"
              >
                <div className="font-semibold">{rootTask}</div>
              </div>
            ))}
          </div>
        </div>

        {Array.from(selectedDirectorRobot?.providers.keys() || []).map((layerName: string) => (
          <div key={layerName} className="m-4 bg-auto-surface-2 p-4 border border-2 border-auto rounded">
            <div className="text-lg bg-auto-surface-1 py-1 px-4 w-fit h-fit rounded mb-4">{layerName}</div>
            <div className="grid grid-cols-2 gap-4">
              <div className="bg-auto-surface-1 p-2 rounded">
                Active
                <div className="flex flex-wrap">
                  {selectedDirectorRobot?.providers
                    .get(layerName)
                    ?.filter((provider) => provider.active)
                    .map((provider) => (
                      <div
                        key={provider.id}
                        className={`bg-auto-surface-1 flex min-w-[7em] w-fit h-fit p-4 m-1 rounded border border-2 border-auto ${provider.done ? "bg-green-100" : ""
                          }`}
                      >
                        <div>
                          <div className="font-semibold">{provider.name}</div>
                          <div className="text-xs">{provider.done ? " (Done)" : "(Not Done)"}</div>
                        </div>
                      </div>
                    ))}
                </div>
              </div>
              <div className="bg-auto-surface-1 p-2 rounded">
                Not Active
                <div className="flex flex-wrap">
                  {selectedDirectorRobot?.providers
                    .get(layerName)
                    ?.filter((provider) => !provider.active)
                    .map((provider) => (
                      <div
                        key={provider.id}
                        className={`bg-auto-surface-1 flex min-w-[7em] w-fit h-fit p-4 m-1 rounded border border-2 border-auto border-dashed ${provider.done ? "bg-green-100" : ""
                          }`}
                      >
                        <div>
                          <div className="font-semibold">{provider.name}</div>
                          <div className="text-xs">{provider.done ? " (Done)" : "(Not Done)"}</div>
                        </div>
                      </div>
                    ))}
                </div>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
});
