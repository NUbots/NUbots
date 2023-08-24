import { NUClearNetPacket, NUClearNetSend } from "nuclearnet.js";
import { NUClearNetPeer } from "nuclearnet.js";

import { compose } from "../../shared/base/compose";
import { NUClearNetClient, NUClearPacketMetadata } from "../../shared/nuclearnet/nuclearnet_client";
import { NbsPacketProcessor } from "../nbs_scrubber/packet_processor";
import { ScrubberSet } from "../nbs_scrubber/scrubber_set";
import { NUClearNetPacketProcessor } from "../nuclearnet/packet_processor";
import { ClientConnection } from "../web_socket/client_connection";

import { NUsightSession } from "./session";

/** Represents a NUsight browser client and handles communication with the client */
export class NUsightSessionClient {
  /**
   * The registered events that this client is listening for. Maps event names to
   * event registration objects which provide a means of cancelling the event listeners.
   */
  private readonly onEvents = new Map<string, { event: string; off: () => void }>();

  constructor(
    public readonly id: number,
    public connection: ClientConnection,
    private nuclearnetClient: NUClearNetClient,
    private nuclearNetPacketProcessor: NUClearNetPacketProcessor,
    private nbsPacketProcessor: NbsPacketProcessor,
    private session: NUsightSession,
  ) {}

  static of(
    id: number,
    connection: ClientConnection,
    opts: {
      nuclearnetClient: NUClearNetClient;
      scrubberSet: ScrubberSet;
      session: NUsightSession;
    },
  ) {
    return new NUsightSessionClient(
      id,
      connection,
      opts.nuclearnetClient,
      NUClearNetPacketProcessor.of((event: string, ...args: any[]) => {
        connection.send(event, ...args);
      }),
      NbsPacketProcessor.of(opts.scrubberSet, (event: string, ...args: any[]) => {
        connection.send(event, ...args);
      }),
      opts.session,
    );
  }

  /**
   * Start listening for the events we need to process for the client,
   * to handle messages to and from the client.
   */
  startProcessing(): () => void {
    return compose([
      this.nuclearnetClient.onJoin(this.onJoin),
      this.nuclearnetClient.onLeave(this.onLeave),
      this.connection.on("listen", this.onListen),
      this.connection.on("unlisten", this.onUnlisten),
      this.connection.on("packet", this.onClientPacket),
      () => {
        this.onEvents.forEach((event) => event.off());
        this.onEvents.clear();
        this.nbsPacketProcessor.destroy();
      },
    ]);
  }

  /** Handle a new NUClearNet peer joining by forwarding the join to the client */
  private onJoin = (peer: NUClearNetPeer) => {
    this.connection.send("nuclear_join", peer);
  };

  /** Handle a new NUClearNet peer leaving by forwarding the leave to the client */
  private onLeave = (peer: NUClearNetPeer) => {
    this.connection.send("nuclear_leave", peer);
  };

  /** Handle a request from the client to listen for a message */
  private onListen = (event: string, requestToken: string) => {
    // Listen for the message on NUClearNet
    const nuclearNetOff = this.nuclearnetClient.on(event, this.onServerPacket.bind(this, event));

    // Add the event type to the NBS packet processor, to forward
    // packets of that type to the client from loaded scrubbers
    const nbsScrubberOff = this.nbsPacketProcessor.listenFor(event);

    // Keep track of the offs in an event registration so we can cancel when necessary
    this.onEvents.set(requestToken, {
      event,
      off() {
        nuclearNetOff();
        nbsScrubberOff();
      },
    });
  };

  /** Handle a request from the client to stop listening for a message */
  private onUnlisten = (requestToken: string) => {
    // Get the event registration for this request token
    const eventRegistration = this.onEvents.get(requestToken);

    // If we have an event registration, cancel and remove it
    if (eventRegistration) {
      eventRegistration.off();
      this.onEvents.delete(requestToken);
    }
  };

  /** Handle an incoming packet from NUClearNet by forwarding it to the client */
  private onServerPacket = (event: string, packet: NUClearNetPacket, packetMetadata?: NUClearPacketMetadata) => {
    // Add the packet to the client's packet processing queue, so it's sent when appropriate
    this.nuclearNetPacketProcessor.onPacket(event, packet, packetMetadata);
  };

  /** Handle an outgoing packet from the client by forwarding it to NUClearNet */
  private onClientPacket = (options: NUClearNetSend) => {
    // Handle packets sent to the NUsight server
    if (options.target === "nusight") {
      this.session.processClientPacket(options, this);
    }
    // Forward all other packets to NUClearNet
    else {
      this.nuclearnetClient.send(options);
    }
  };
}
