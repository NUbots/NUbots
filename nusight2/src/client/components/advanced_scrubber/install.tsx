import React from "react";
import { NavigationConfiguration } from "@client/navigation";
import { NUsightNetwork } from "@client/network/nusight_network";
import { AppModel } from "@components/app/model";

function IconScrubber(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      viewBox="0 0 24 24"
      width="24"
      height="24"
      fill="currentColor"
      xmlns="http://www.w3.org/2000/svg"
    >
      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 14.5v-9l6 4.5-6 4.5z" />
    </svg>
  );
}

export function installStandaloneAdvancedScrubber({
  nav,
  appModel,
  nusightNetwork,
}: {
  nav: NavigationConfiguration;
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
}) {
  nav.addRoute({
    path: "/scrubber",
    Icon: IconScrubber,
    label: "Scrubber",
    Content: React.lazy(async () => {
      const { createStandaloneAdvancedScrubberView } = await import("./create");
      return {
        default: createStandaloneAdvancedScrubberView({ appModel, nusightNetwork }),
      };
    }),
  });
}
