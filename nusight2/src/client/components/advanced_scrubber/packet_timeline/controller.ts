import { NbsScrubberController } from "@components/nbs_scrubbers/nbs_scrubber/controller";
import { TimelineDataPoint } from "@components/timeline/model";
import { RpcNetwork } from "@hooks/use_rpc_controller";
import { ScrubberPacketRequest } from "@proto/message/eye/Scrubber";
import { clamp } from "@shared/math/clamp";
import { messageNameToType } from "@shared/messages/generated/type_converters";
import { Timestamp } from "@shared/time/timestamp";
import { action } from "mobx";

import { MessageTypeId } from "../model";

import { PacketTimelineModel } from "./model";

export class PacketTimelineController {
  constructor(
    private network: RpcNetwork,
    private model: PacketTimelineModel,
    private scrubberController: NbsScrubberController,
  ) {}

  seek(timestamp: bigint) {
    const clampedTimestamp = clamp(timestamp, this.model.scrubber.start, this.model.scrubber.end);
    this.scrubberController.seekToTimestamp(clampedTimestamp);
  }

  @action.bound
  async selectPacket(type: MessageTypeId, offset: number, dataPoint: TimelineDataPoint) {
    const { typeHash, subtype } = type;
    const result = await this.network.call(
      new ScrubberPacketRequest({ id: this.model.scrubber.id, typeHash, subtype, offset }),
    );

    if (result.ok) {
      this.setSelectedMessage(type, result.data.response.data, dataPoint);
    } else {
      // TODO: add toast messages to NUsight to show errors like this without using alert()
      alert(result.error.message);
    }
  }

  @action.bound
  deselectPacket() {
    this.model.highlightedDataPoints = [];
    this.model.selectedMessage = undefined;
  }

  @action
  private setSelectedMessage(type: MessageTypeId, payload: Uint8Array, dataPoint: TimelineDataPoint) {
    this.model.highlightedDataPoints = [dataPoint];
    this.scrubberController.seekToTimestamp(dataPoint.timestamp);
    try {
      this.model.selectedMessage = {
        type,
        timestamp: new Timestamp(dataPoint.timestamp),
        message: messageNameToType(type.typeName).fromBinary(payload),
        size: payload.length,
      };
    } catch {
      // Fallback to showing the raw bytes if decoding failed
      this.model.selectedMessage = {
        type,
        timestamp: new Timestamp(dataPoint.timestamp),
        message: payload,
        size: payload.length,
      };
    }
  }
}
