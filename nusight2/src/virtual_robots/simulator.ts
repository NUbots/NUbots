import { NUClearNetClient } from "../shared/nuclearnet/nuclearnet_client";

export interface Message {
  messageType: string;
  buffer: Uint8Array;
  reliable?: boolean;
}

export abstract class Simulator {
  constructor(protected readonly nuclearnetClient: NUClearNetClient) {}

  protected send(message: Message): void {
    this.nuclearnetClient.send({
      type: message.messageType,
      payload: Buffer.from(message.buffer),
      target: "nusight",
      reliable: message.reliable,
    });
  }

  abstract start(): () => void;
}
