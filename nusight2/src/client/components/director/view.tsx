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
                <div className="my-4 bg-gray-50 p-4 border border-2 border-gray-400 rounded">
                    <div className="text-lg bg-gray-200 py-1 px-4 w-fit rounded mb-4 mx-1">message::skill</div>
                    <div className="grid grid-cols-2 gap-4">
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Active, Not Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::skill") && provider.active && !provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::skill::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Active, Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::skill") && provider.active && provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::skill::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Inactive, Not Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::skill") && !provider.active && !provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::skill::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Inactive, Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::skill") && !provider.active && provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::skill::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                    </div>
                </div>
                <div className="my-4 bg-gray-50 p-4 border border-2 border-gray-400 rounded">
                    <div className="text-lg bg-gray-200 py-1 px-4 w-fit rounded mb-4 mx-1">message::actuation</div>
                    <div className="grid grid-cols-2 gap-4">
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Active, Not Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::actuation") && provider.active && !provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::actuation::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Active, Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::actuation") && provider.active && provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::actuation::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Inactive, Not Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::actuation") && !provider.active && !provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::actuation::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                        <div className="bg-gray-200 border-gray-400 border border-2 p-3 rounded ">
                            Inactive, Done
                            <div className="flex flex-wrap">
                                {selectedDirectorRobot?.providers
                                    .filter(provider => provider.name.startsWith("message::actuation") && !provider.active && provider.done)
                                    .map(provider => (
                                        <div key={provider.id} className="bg-gray-100 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                            <div>
                                                {provider.name.replace("message::actuation::", "")}
                                            </div>
                                        </div>
                                    ))}</div>
                        </div>
                    </div>
                </div>
                {/* <div className="my-4 bg-blue-50 p-4 border border-2 border-blue-400 rounded">
                    <div className="text-lg bg-blue-100 p-1 w-fit rounded mb-4 mx-1">message::planning</div>
                    <div className="bg-green-200 w-50% flex">
                        {selectedDirectorRobot?.providers
                            .filter(provider => provider.name.startsWith("message::planning") && provider.active)
                            .map(provider => (
                                <div key={provider.id} className="bg-green-200 flex w-fit p-4 m-1 rounded border border-2 border-gray-500">
                                    <div>
                                        {provider.name.replace("message::skill", "")}
                                    </div>
                                </div>
                            ))}
                    </div>
                </div> */}
            </div>
        </div >
    );
});
