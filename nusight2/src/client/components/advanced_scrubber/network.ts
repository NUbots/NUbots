import { Network } from "@client/network/network";
import { RobotModel } from "@components/robot/model";
import { ScrubberIndex } from "@proto/message/eye/Scrubber";
import { Timestamp } from "@shared/time/timestamp";
import { action } from "mobx";

import { AdvancedScrubberScrubberModel } from "./model";

export class AdvancedScrubberNetwork {
  constructor(
    private model: AdvancedScrubberScrubberModel,
    private network: Network,
  ) {
    this.network.on({ type: ScrubberIndex, subtype: model.scrubber.id }, this.onScrubberIndex);
  }

  destroy() {
    this.network.off();
  }

  @action
  private onScrubberIndex = (_: RobotModel, message: ScrubberIndex) => {
    this.model.indices = message.indices.map((index) => ({
      type: {
        typeName: index.typeName!,
        typeHash: index.typeHash!,
        subtype: index.subtype!,
      },
      timestamps: index.timestamps?.map((t) => Timestamp.toNanos(t)) ?? [],
    }));
  };
}
