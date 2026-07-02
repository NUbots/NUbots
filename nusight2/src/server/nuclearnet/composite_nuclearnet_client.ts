import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";

import { compose } from "../../shared/base/compose";
import { NUClearEventListener } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearPacketListener } from "../../shared/nuclearnet/nuclearnet_client";

/**
 * A {@link NUClearNetClient} that fans every operation out to a primary client and one or more
 * secondary clients. Peers and packets from any of the underlying clients are surfaced as if they
 * came from a single client.
 *
 * This is used to combine the real NUClearNet connection with additional receive-only sources such
 * as the {@link ./overview_udp_client.OverviewUDPClient Overview UDP side channel}.
 *
 * Sending is delegated to the primary client only, since the secondary clients are receive-only.
 */
export class CompositeNUClearNetClient implements NUClearNetClient {
  /** All underlying clients, with the primary client first. */
  private readonly clients: NUClearNetClient[];

  constructor(
    private readonly primary: NUClearNetClient,
    secondaries: NUClearNetClient[],
  ) {
    this.clients = [primary, ...secondaries];
  }

  static of(primary: NUClearNetClient, secondaries: NUClearNetClient[]): CompositeNUClearNetClient {
    return new CompositeNUClearNetClient(primary, secondaries);
  }

  connect(options: NUClearNetOptions): () => void {
    return compose(this.clients.map((client) => client.connect(options)));
  }

  onJoin(cb: NUClearEventListener): () => void {
    return compose(this.clients.map((client) => client.onJoin(cb)));
  }

  onLeave(cb: NUClearEventListener): () => void {
    return compose(this.clients.map((client) => client.onLeave(cb)));
  }

  on(event: string, cb: NUClearPacketListener): () => void {
    return compose(this.clients.map((client) => client.on(event, cb)));
  }

  onPacket(cb: NUClearPacketListener): () => void {
    return compose(this.clients.map((client) => client.onPacket(cb)));
  }

  send(options: NUClearNetSend): void {
    this.primary.send(options);
  }
}
