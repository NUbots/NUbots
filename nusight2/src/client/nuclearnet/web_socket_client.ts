export interface WebSocketClient {
  connect(): void;

  disconnect(): void;

  on(event: string, fn: Function): void;

  off(event: string, fn?: Function): void;

  send(event: string, ...args: any[]): void;
}
