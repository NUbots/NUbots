import { NUClearNetOptions, NUClearNetPeer } from "nuclearnet.js";

import { compose } from "../shared/base/compose";
import { NUClearNetClient } from "../shared/nuclearnet/nuclearnet_client";

import { DirectNUClearNetClient } from "./nuclearnet/direct_nuclearnet_client";
import { FakeNUClearNetClient } from "./nuclearnet/fake_nuclearnet_client";
import { NUsightSession } from "./session/session";
import { ClientConnection } from "./web_socket/client_connection";
import { WebSocketServer } from "./web_socket/web_socket_server";

interface NUsightServerOpts {
  fakeNetworking: boolean;
  connectionOpts: NUClearNetOptions;
}

/**
 * The server component of a running NUsight instance. Provides the following services for NUsight browser clients connected via websocket:
 *   - Acts as a gateway to the NUClear network. All clients currently share a single NUClearNet connection, mostly for
 *     performance reasons. Could potentially be improved to have more intelligent multiplexing.
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
    private readonly nuclearnetClient: NUClearNetClient,
    private readonly connectionOpts: NUClearNetOptions,
  ) {
    // Listen for websocket connections to add new clients when they connect
    websocketServer.onConnection(this.onClientConnection);

    // Connect to NUClearNet
    this.destroy = compose([
      this.nuclearnetClient.onJoin(this.onPeerJoin),
      this.nuclearnetClient.onLeave(this.onPeerLeave),
      this.nuclearnetClient.connect(this.connectionOpts),
      () => {
        // Close all sessions when this server is destroyed
        for (const session of this.sessions.values()) {
          session.destroy();
        }
        this.sessions.clear();
      },
    ]);
  }

  static of(server: WebSocketServer, { fakeNetworking, connectionOpts }: NUsightServerOpts): NUsightServer {
    const nuclearnetClient: NUClearNetClient = fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of();
    return new NUsightServer(server, nuclearnetClient, connectionOpts);
  }

  /** Handle a new client connection */
  private onClientConnection = (connection: ClientConnection) => {
    // TODO: calculate a session id based on the client's browsing context,
    // so multiple tabs/windows can share a single NUsight session.
    const sessionId = "todo-replace-with-clients-session-id";

    // Get the session or create a new one if it doesn't exist
    const session = this.sessions.get(sessionId) ?? NUsightSession.of(this.nuclearnetClient);
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
