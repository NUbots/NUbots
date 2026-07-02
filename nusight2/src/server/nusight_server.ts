import { NUClearNetOptions, NUClearNetPeer } from "nuclearnet.js";

import { compose } from "../shared/base/compose";
import { NUClearNetClient } from "../shared/nuclearnet/nuclearnet_client";

import { DirectNUClearNetClient } from "./nuclearnet/direct_nuclearnet_client";
import { FakeNUClearNetClient } from "./nuclearnet/fake_nuclearnet_client";
import { RoboCupUDPClient, RoboCupUDPClientOptions } from "./nuclearnet/robocup_udp_client";
import { NUsightSession } from "./session/session";
import { ClientConnection } from "./web_socket/client_connection";
import { WebSocketServer } from "./web_socket/web_socket_server";

interface NUsightServerOpts {
  fakeNetworking: boolean;
  connectionOpts: NUClearNetOptions;
  /**
   * When provided, an additional receive-only UDP side channel is opened that presents robots
   * (or other teams) sending serialised RoboCup team communication packets as their own peers.
   */
  robocupUDP?: RoboCupUDPClientOptions;
}

/**
 * The server component of a running NUsight instance. Provides the following services for NUsight browser clients connected via websocket:
 *   - Acts as a gateway to one or more independent network sources (the real NUClearNet connection, plus any UDP side
 *     channels such as RoboCup team communication). Each source is a separate `NUClearNetClient` in its own right —
 *     none of them are "the main one" — and every peer they report keeps its own identity, so e.g. a robot connected
 *     over NUClearNet and a robot only visible via the RoboCup UDP side channel show up as distinct peers.
 *   - Provides NBS playback and scrubbing for clients connected in a "session". A session is a group of related clients
 *     that share a common set of NBS scrubbers.
 */
export class NUsightServer {
  /** The currently connected NUClearNet peers */
  private readonly peers = new Set<NUClearNetPeer>();

  /** The currently running NUsight sessions */
  private readonly sessions = new Map<string, NUsightSession>();

  /** Disconnect from NUClearNet and clean up used resources */
  public destroy?: () => void;

  constructor(
    readonly websocketServer: WebSocketServer,
    private readonly nuclearnetClients: NUClearNetClient[],
    private readonly connectionOpts: NUClearNetOptions,
  ) {
    // Listen for websocket connections to add new clients when they connect
    websocketServer.onConnection(this.onClientConnection);

    // Connect every network source and track peers joining/leaving across all of them
    this.destroy = compose([
      ...this.nuclearnetClients.flatMap((client) => [client.onJoin(this.onPeerJoin), client.onLeave(this.onPeerLeave)]),
      ...this.nuclearnetClients.map((client) => client.connect(this.connectionOpts)),
      () => {
        // Close all sessions when this server is destroyed
        for (const session of this.sessions.values()) {
          session.destroy();
        }
        this.sessions.clear();
      },
    ]);
  }

  static of(server: WebSocketServer, { fakeNetworking, connectionOpts, robocupUDP }: NUsightServerOpts): NUsightServer {
    // Every network source NUsight combines together. There's no single "main" client here — each
    // is an independent connection whose peers and packets are surfaced as their own distinct
    // peers (see robocup_udp_client.ts for how the RoboCup UDP side channel identifies itself).
    const nuclearnetClients: NUClearNetClient[] = [
      fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of(),
    ];

    if (robocupUDP) {
      nuclearnetClients.push(RoboCupUDPClient.of(robocupUDP));
    }

    return new NUsightServer(server, nuclearnetClients, connectionOpts);
  }

  /** Handle a new client connection */
  private onClientConnection = (connection: ClientConnection) => {
    // TODO: calculate a session id based on the client's browsing context,
    // so multiple tabs/windows can share a single NUsight session.
    const sessionId = "todo-replace-with-clients-session-id";

    // Get the session or create a new one if it doesn't exist
    const session = this.sessions.get(sessionId) ?? NUsightSession.of(this.nuclearnetClients);
    this.sessions.set(sessionId, session);

    // Add a client to the session, with the new connection
    session.addClient(connection);

    // Send the new client fake join packets for everyone already on the network
    for (const peer of this.peers) {
      connection.send("nuclear_join", peer);
    }
  };

  /** Keep track of NUClearNet peers that join */
  private onPeerJoin = (peer: NUClearNetPeer) => {
    console.info(`Connected to ${peer.name} on ${peer.address}:${peer.port}`);
    this.peers.add(this.getCanonicalPeer(peer));
  };

  /** Remove NUClearNet peers that leave */
  private onPeerLeave = (peer: NUClearNetPeer) => {
    console.info(`Disconnected from ${peer.name} on ${peer.address}:${peer.port}`);
    this.peers.delete(this.getCanonicalPeer(peer));
  };

  /**
   * NUClearNet peer objects do not maintain referential identity, this normalizes them so that they do.
   * This allows them to be used within contexts that require triple equal (===) object equality (e.g. sets).
   */
  private getCanonicalPeer(peer: NUClearNetPeer): NUClearNetPeer {
    const existingPeer = Array.from(this.peers.values()).find(
      (otherPeer) => otherPeer.name === peer.name && otherPeer.address === peer.address && otherPeer.port === peer.port,
    );
    return existingPeer ?? peer;
  }
}
