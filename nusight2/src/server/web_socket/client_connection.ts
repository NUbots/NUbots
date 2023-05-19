/**
 * Interface for a connection to a NUsight client, abstracted here to allow for mock
 * implementations when testing. Currently has one implementation: WebSocket.
 */
export interface ClientConnection {
  /** Register a callback to be called when the connection is disconnected */
  onDisconnect(cb: (reason: string) => void): void;

  /** Register a callback to be called when a message is received from the client */
  on(event: string, cb: (...args: any[]) => void): () => void;

  /** Send a message to the client */
  send(event: string, ...args: any[]): void;
}
