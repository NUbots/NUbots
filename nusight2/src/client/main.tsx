import "./tailwind.css";

import React from "react";
import { configure } from "mobx";
import { createRoot } from "react-dom/client";

import { setAppContext } from "./components/app/context";
import { AppController } from "./components/app/controller";
import { AppModel } from "./components/app/model";
import { AppNetwork } from "./components/app/network";
import { AppView } from "./components/app/view";
import { installChart } from "./components/chart/install";
import { installDashboard } from "./components/dashboard/install";
import { installLocalisation } from "./components/localisation/install";
import { installLogs } from "./components/logs/install";
import { withRobotSelectorMenuBar } from "./components/menu_bar/view";
import { installOdometry } from "./components/odometry/install";
import { installProfiler } from "./components/profiler/install";
import { installVision } from "./components/vision/install";
import { installVisualMesh } from "./components/visual_mesh/install";
import { NavigationConfiguration } from "./navigation";
import { NUsightNetwork } from "./network/nusight_network";

const nav = NavigationConfiguration.of();
const appModel = AppModel.of();
const nusightNetwork = NUsightNetwork.of(appModel);
nusightNetwork.connect({ name: "nusight" });

setAppContext({ nusightNetwork });

const appController = AppController.of();
AppNetwork.of(nusightNetwork, appModel);
const Menu = withRobotSelectorMenuBar(appModel, appController.toggleRobotEnabled);

installDashboard({ nav, appModel, nusightNetwork, Menu });
installLocalisation({ nav, appModel, nusightNetwork, Menu });
installOdometry({ nav, appModel, nusightNetwork, Menu });
installChart({ nav, appModel, nusightNetwork, Menu });
installVision({ nav, appModel, nusightNetwork, Menu });
installVisualMesh({ nav, appModel, nusightNetwork, Menu });
installProfiler({ nav, appModel, nusightNetwork, Menu });
installLogs({ nav, appModel, nusightNetwork, Menu });

configure({ enforceActions: "observed" });
createRoot(document.getElementById("root")!).render(<AppView nav={nav} />);
