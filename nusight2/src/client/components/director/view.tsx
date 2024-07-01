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

    console.log("DirectorView", selectedDirectorRobot)
    // console.log("Provider", selectedDirectorRobot?.providers[0])


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
                Providers:
                {selectedDirectorRobot?.providers
                    .filter(provider => provider.name.startsWith("message::skill") && provider.active)
                    .map(provider => (
                        <div key={provider.id} className="bg-green-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                            {provider.name}
                        </div>
                    ))}
                <div className="my-4 bg-blue-50 p-4 border border-2 border-blue-400 rounded">
                    <h1 className="text-lg">Skill</h1>
                    <div className="flex-wrap flex">
                        <div className="my-4 bg-blue-100 p-4 border border-2 border-blue-400 rounded">
                            <h2 className="text-md">Active</h2>
                            <div className="flex-wrap flex">
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-green-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-green-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessasdfgeName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> Mesdf sad fssageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                            </div>

                        </div>
                        <div className="my-4 bg-blue-100 p-4 border border-2 border-gray-400 rounded">
                            <h2 className="text-md">Inactive</h2>
                            <div className="flex-wrap flex">
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> Mesdf sad fssageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                                <div className="bg-gray-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                            </div>

                        </div>
                    </div>
                </div>
                <div className="my-4 bg-blue-100 p-4 border border-2 border-blue-400 rounded">
                    <h1 className="text-lg">Planning</h1>
                    <div className="flex-wrap flex">
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessasdfgeName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> Mesdf sad fssageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> sdfa</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessaadgeName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> Messsadf saageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessaadfgeName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                        <div className="bg-blue-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500"> MessageName</div>
                    </div>
                </div>
            </div>
        </div >
    );
});
