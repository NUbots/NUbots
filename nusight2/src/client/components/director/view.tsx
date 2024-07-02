import React from "react";
import { PropsWithChildren } from "react";
import { observer } from "mobx-react";

import { RobotSelectorSingle } from "../robot_selector_single/view";

import { DirectorController } from "./controller";
import { DirectorModel } from "./model";

interface DirectorViewProps {
    controller: DirectorController;
    model: DirectorModel;
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
                <div className="m-4 bg-gray-50 p-4 border border-2 border-gray-400 rounded">
                    <div className="text-lg bg-gray-200 py-1 px-4 w-fit h-fit rounded mb-4">Root Tasks</div>
                    <div className="flex flex-wrap">
                        {Array.from(selectedDirectorRobot?.rootTasks || []).map((rootTask) => (
                            <div key={rootTask} className="bg-gray-100 flex w-fit h-fit p-4 m-1 rounded border border-2 border-gray-500">
                                <div>
                                    {rootTask}
                                </div>
                            </div>
                        ))}
                    </div>
                </div>

                {Array.from(selectedDirectorRobot?.providers.keys() || []).map((layerName: string) => (
                    <div key={layerName} className="m-4 bg-gray-50 p-4 border border-2 border-gray-400 rounded">
                        <div className="text-lg bg-gray-200 py-1 px-4 w-fit h-fit rounded mb-4">{layerName}</div>
                        <div className="grid grid-cols-2 gap-4">
                            <div className="bg-gray-200 p-2 rounded">
                                Active, Not Done
                                <div className="flex flex-wrap">
                                    {selectedDirectorRobot?.providers.get(layerName)?.filter(provider => provider.active && !provider.done).map((provider) => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit h-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name}
                                            </div>
                                        </div>
                                    ))}
                                </div>
                            </div>
                            <div className="bg-gray-200 p-2 rounded">
                                Active, Done
                                <div className="flex flex-wrap">
                                    {selectedDirectorRobot?.providers.get(layerName)?.filter(provider => provider.active && provider.done).map((provider) => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit h-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name}
                                            </div>
                                        </div>
                                    ))}
                                </div>
                            </div>
                            <div className="bg-gray-200 p-2 rounded ">
                                Not Active, Not Done
                                <div className="flex flex-wrap">
                                    {selectedDirectorRobot?.providers.get(layerName)?.filter(provider => !provider.active && !provider.done).map((provider) => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit h-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name}
                                            </div>
                                        </div>
                                    ))}
                                </div>
                            </div>
                            <div className="bg-gray-200 p-2 rounded ">
                                Not Active, Done
                                <div className="flex flex-wrap">
                                    {selectedDirectorRobot?.providers.get(layerName)?.filter(provider => !provider.active && provider.done).map((provider) => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit h-fit h-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name}
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
