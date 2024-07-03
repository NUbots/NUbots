import React, { useContext } from "react";

import { NUsightNetwork } from "../../network/nusight_network";

interface AppContext {
  nusightNetwork: NUsightNetwork;
}

let appContext: React.Context<AppContext> | undefined;

export function setAppContext(contextOpts: AppContext) {
  appContext = React.createContext<AppContext>(contextOpts);
}

export function useNUsightNetwork() {
  // Ensure that the app context is set before attempting to use it
  if (!appContext) {
    throw new Error("Unable to get NUsightNetwork: appContext is not set");
  }

  return useContext(appContext).nusightNetwork;
}
