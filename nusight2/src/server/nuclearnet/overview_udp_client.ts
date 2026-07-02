import { Overview } from "@proto/message/support/nusight/Overview";

import { hashType } from "../../shared/nuclearnet/hash_type";
import { NUClearNetPeerWithType } from "../../shared/nuclearnet/nuclearnet_client";

import { DecodedUDPMessage } from "./udp_side_channel_client";
import { UDPSideChannelClient } from "./udp_side_channel_client";
import { UDPSideChannelClientOptions } from "./udp_side_channel_client";

/** The fully qualified Protobuf type name for the Overview message. */
const OVERVIEW_TYPE = "message.support.nusight.Overview";

/** The peer type presented for connections made via the {@link OverviewUDPClient}. */
export const OVERVIEW_UDP_PEER_TYPE: NUClearNetPeerWithType["type"] = "overview-udp-peer";

export type OverviewUDPClientOptions = UDPSideChannelClientOptions;

/**
 * A receive-only {@link NUClearNetClient} that listens on a UDP port for serialised Overview
 * messages. Each unique sender is presented as its own NUClearNet-style peer (tagged as an
 * {@link OVERVIEW_UDP_PEER_TYPE}, distinct from regular NUClearNet peers) that only ever
 * delivers the single Overview packet it sent.
 *
 * This provides a low bandwidth "side channel" for robots that cannot join the NUClear network
 * directly (e.g. across a restrictive network) but can still send a single UDP packet.
 */
export class OverviewUDPClient extends UDPSideChannelClient {
  private readonly hash = hashType(OVERVIEW_TYPE);

  constructor(options: OverviewUDPClientOptions) {
    super(options, OVERVIEW_UDP_PEER_TYPE);
  }

  static of(options: OverviewUDPClientOptions): OverviewUDPClient {
    return new OverviewUDPClient(options);
  }

  /** Decode an incoming UDP datagram containing a serialised Overview message. */
  protected decode(payload: Buffer, address: string, port: number): DecodedUDPMessage | undefined {
    let robotId = 0;
    let roleName = "";
    try {
      const overview = Overview.fromBinary(new Uint8Array(payload));
      robotId = overview.robotId ?? 0;
      roleName = overview.roleName ?? "";
    } catch (error) {
      console.warn(`Overview UDP side channel: failed to decode packet from ${address}:${port}`, error);
      return undefined;
    }

    return {
      key: `${address}:${robotId}`,
      name: roleName !== "" ? roleName : `overview-${robotId}`,
      packets: [{ type: OVERVIEW_TYPE, hash: this.hash, payload }],
    };
  }
}
