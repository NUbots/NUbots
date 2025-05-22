import { NUClearNetSend } from "nuclearnet.js";

import { compose } from "../../shared/base/compose";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { createFilePicker } from "../file_picker/create";
import { createNbsScrubber } from "../nbs_scrubber/create";
import { ScrubberSet } from "../nbs_scrubber/scrubber_set";
import { ClientConnection } from "../web_socket/client_connection";

import { NUsightSessionClient } from "./session_client";
import { NUsightSessionNetwork } from "./session_network";

type ClientPacketCallback = (client: NUsightSessionClient, packet: NUClearNetSend) => void;
type ServerModuleCreator = (session: NUsightSession) => () => void;

/** Used for generating IDs for session clients */
let nextClientId = 1;

/**
 * Represents a "session": a collection of related NUsight browser clients
 * that share a common set of NBS scrubbers.
 */
export class NUsightSession {
  /** The browser clients currently connected to this session */
  readonly clients = new Set<NUsightSessionClient>();

  /**
   * A NUClearNet client connection used for sending and receiving messages
   * by clients in this session.
   */
  readonly nuclearnetClient: NUClearNetClient;

  /** Holds the scrubbers loaded by clients in this session */
  readonly scrubberSet: ScrubberSet;

  /**
   * The network for this session: allows for exchanging messages
   * between the server, clients in the session, and NUClearNet
   */
  readonly network: NUsightSessionNetwork;

  /** Map of client packet event names to listener callbacks */
  private clientPacketOns = new Map<string, Set<ClientPacketCallback>>();

  /** Clean up functions for the connected clients, run when a client disconnects */
  private clientCleanups = new Map<NUsightSessionClient, () => void>();

  /** Clean up function for modules installed into the session */
  private cleanUpModules: () => void;

  /** Create a new NUsightSession with the given NUClearNet connection */
  constructor(nuclearnetClient: NUClearNetClient, moduleCreators: ServerModuleCreator[]) {
    this.nuclearnetClient = nuclearnetClient;
    this.scrubberSet = ScrubberSet.of();
    this.network = new NUsightSessionNetwork(this);

    // Install the server-side modules for the session
    this.cleanUpModules = compose(moduleCreators.map((createModule) => createModule(this)));
  }

  static of(nuclearnetClient: NUClearNetClient) {
    return new NUsightSession(nuclearnetClient, [createNbsScrubber, createFilePicker]);
  }

  /** Add a client for the given connection to the session */
  addClient(clientConnection: ClientConnection): Readonly<NUsightSessionClient> {
    // Create and add a client for the connection
    const client = NUsightSessionClient.of(nextClientId++, clientConnection, {
      nuclearnetClient: this.nuclearnetClient,
      scrubberSet: this.scrubberSet,
      session: this,
    });

    this.clients.add(client);

    // Start processing messages for the client
    const stopProcessing = client.startProcessing();

    // Create a cleanup function for the client
    const cleanupClient = () => {
      stopProcessing();
      this.clients.delete(client);
      this.clientCleanups.delete(client);
    };

    // Clean up when the client disconnects
    clientConnection.onDisconnect(cleanupClient);

    // Store the cleanup function so we can call it when the client is removed
    // or when the session is destroyed
    this.clientCleanups.set(client, cleanupClient);

    // Send the client all currently loaded scrubbers in this session
    for (const scrubber of this.scrubberSet.scrubbers.values()) {
      clientConnection.send("nuclear_join", scrubber.peer);
    }

    return client;
  }

  /** Close the session and clean up all client connections and scrubbers */
  destroy() {
    this.clientPacketOns.clear();

    for (const cleanup of this.clientCleanups.values()) {
      cleanup();
    }

    this.clientCleanups.clear();
    this.clients.clear();

    this.cleanUpModules();

    this.scrubberSet.destroy();
  }

  /** Send the given message to all clients in this session */
  sendToAll(event: string, ...args: any[]) {
    for (const client of this.clients) {
      client.connection.send(event, ...args);
    }
  }

  /** Send the given message to all clients in this session except the given client */
  sendToOthers(self: NUsightSessionClient, event: string, ...args: any[]) {
    for (const client of this.clients) {
      if (client !== self) {
        client.connection.send(event, ...args);
      }
    }
  }

  /** Register a listener for a packet of the given event type from a client */
  onClientPacket(event: string, callback: ClientPacketCallback) {
    const callbacks = this.clientPacketOns.get(event) ?? new Set();

    callbacks.add(callback);

    this.clientPacketOns.set(event, callbacks);

    return () => {
      callbacks.delete(callback);
    };
  }

  /** Process a packet received from a client */
  processClientPacket(options: NUClearNetSend, client: NUsightSessionClient) {
    if (typeof options.type !== "string") {
      console.error("Unable to process client packet where type is not a string", options);
      return;
    }

    const callbacks = this.clientPacketOns.get(options.type);

    if (!callbacks) {
      return;
    }

    for (const callback of callbacks) {
      callback(client, options);
    }
  }
}
