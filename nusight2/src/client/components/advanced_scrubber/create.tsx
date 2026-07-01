import React, { useEffect, useMemo } from "react";
import { Network } from "@client/network/network";
import { NUsightNetwork } from "@client/network/nusight_network";
import { AppModel } from "@components/app/model";
import { NoSelectedDataSource } from "@components/blank_state";
import { useAutoSelectDataSourceFromUrl } from "@hooks/use_auto_select_data_source";
import { observer } from "mobx-react";

import { AdvancedScrubber } from "./advanced_scrubber";
import { AdvancedScrubberModel, AdvancedScrubberScrubberModel } from "./model";
import { AdvancedScrubberNetwork } from "./network";

export function createStandaloneAdvancedScrubberView({
  appModel,
  nusightNetwork,
}: {
  appModel: AppModel;
  nusightNetwork: NUsightNetwork;
}) {
  const model = AdvancedScrubberModel.of(appModel);

  return observer(() => {
    // Auto select the robot specified in the URL if any.
    useAutoSelectDataSourceFromUrl(model, model.setSelectedRobot, { fallbackToFirst: false });

    return model.selectedScrubber ? (
      <AdvancedScrubberWindow model={model.selectedScrubber} nusightNetwork={nusightNetwork} />
    ) : (
      <NoSelectedDataSource
        hasDataSources={model.robots.length > 0}
        pendingAction="view advanced scrubber"
        urlSelectionRequired
      />
    );
  });
}

interface AdvancedScrubberWindowProps {
  nusightNetwork: NUsightNetwork;
  model: AdvancedScrubberScrubberModel;
}

const AdvancedScrubberWindow = observer(function AdvancedScrubberWindow(props: AdvancedScrubberWindowProps) {
  const { nusightNetwork, model } = props;

  const network = useMemo(() => {
    return new AdvancedScrubberNetwork(model, Network.of(nusightNetwork));
  }, [nusightNetwork]);

  useEffect(() => {
    return () => network.destroy();
  }, [network]);

  useEffect(() => {
    const title = document.title;
    document.title = `${model.scrubber.name} Scrubber — NUsight`;
    return () => {
      document.title = title;
    };
  }, [model.scrubber.name]);

  return <AdvancedScrubber model={model} />;
});
