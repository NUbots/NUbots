import "./tailwind.css";

import React, { Suspense } from "react";
import { configure } from "mobx";
import { createRoot } from "react-dom/client";
import { BrowserRouter, Route, Routes } from "react-router-dom";

import { installStandaloneAdvancedScrubber } from "./components/advanced_scrubber/install";
import { setAppContext } from "./components/app/context";
import { AppModel } from "./components/app/model";
import { AppNetwork } from "./components/app/network";
import { BlankState } from "./components/blank_state";
import { useAutoCloseOnParentClose } from "./hooks/use_auto_close";
import { NavigationConfiguration } from "./navigation";
import { NUsightNetwork } from "./network/nusight_network";
import { usePopoutChild } from "./popouts/popout";

const appModel = AppModel.of();
const nusightNetwork = NUsightNetwork.of(appModel);
nusightNetwork.connect({ name: "nusight" });

setAppContext({ nusightNetwork });

AppNetwork.of(nusightNetwork, appModel);

const nav = NavigationConfiguration.of("/standalone");

// Install the standalone routes
installStandaloneAdvancedScrubber({ nav, appModel, nusightNetwork });

function AppView() {
  // If this is a child window, close it when its parent window closes
  useAutoCloseOnParentClose(location.search.includes("isChild=true"));

  // Connect to the popout parent if this is a child window
  usePopoutChild();

  return (
    <BrowserRouter>
      <div className="bg-gray-900 text-white/90 h-screen w-screen">
        <Suspense fallback={<div className="p-2">Loading...</div>}>
          <Routes>
            {nav.getRoutes().map((route) => (
              <Route key={route.path} path={route.path} element={<route.Content />} />
            ))}
            <Route path="*" element={<BlankState title="404: Not found" icon="warning" />} />
          </Routes>
        </Suspense>
      </div>
    </BrowserRouter>
  );
}

configure({ enforceActions: "observed" });

// Create the React root at most once (even if this module is hot reloaded) and render
const container = document.getElementById("root") as HTMLElement & {
  __reactRoot?: ReturnType<typeof createRoot>;
};
container.__reactRoot = container.__reactRoot ?? createRoot(container);
container.__reactRoot.render(<AppView />);
