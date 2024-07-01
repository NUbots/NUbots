import React from "react";
import { PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Icon } from "../icon/view";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { DirectorController } from "./controller";
import { DirectorModel, DirectorRobotModel } from "./model";

interface DirectorViewProps {
    controller: DirectorController;
    model: DirectorModel;
    Menu: React.ComponentType<PropsWithChildren<any>>;
}

export const DirectorView = observer(function DirectorView(props: DirectorViewProps) {
    const { Menu, model, controller } = props;

    const { robots, selectedDirectorRobot } = model;

    const messageTypes = ["skill", "planning"]

    // console.log("DirectorView", selectedDirectorRobot)
    // // console.log("Provider", selectedDirectorRobot?.providers[0])


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
            <div className="m-10">
                {Array.from(selectedDirectorRobot?.providers.keys() || []).map((layerName: string) => (
                    <div className="my-4 bg-gray-50 p-4 border border-2 border-gray-400 rounded">
                        <div className="text-lg bg-gray-200 py-1 px-4 w-fit h-fit rounded mb-4 mx-1">{layerName}</div>
                        <div className="grid grid-cols-2 gap-4">
                            <div className="bg-gray-200 p-2 rounded ">
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
                            <div className="bg-gray-200 p-2 rounded ">
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
