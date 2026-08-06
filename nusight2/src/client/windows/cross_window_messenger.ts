/** Describes a class of type T which is a message that can be sent across windows */
export interface CrossWindowMessageType<T> {
  type: string;
  new (...args: any[]): T;
}

type CrossWindowMessageCallback<T> = (message: T, source: MessageEventSource | null) => void;

/**
 * Implements an interface for communicating across parent and child windows and windows that share
 * the same [BroadcastChannel](https://developer.mozilla.org/en-US/docs/Web/API/BroadcastChannel).
 */
export class CrossWindowMessenger {
  /** The BroadcastChannel used to communicate between windows that don't share a parent-child relationship */
  broadcastChannel: BroadcastChannel;

  /** Maps message types to registered callbacks for those messages */
  callbacksByType: Record<string, Set<CrossWindowMessageCallback<any>>> = {};

  /** Maps message types to their corresponding message classes, for encoding and decoding */
  typeToMessageClass: Record<string, CrossWindowMessageType<any>> = {};

  constructor(broadcastChannel: BroadcastChannel) {
    this.broadcastChannel = broadcastChannel;

    window.addEventListener("message", this.messageReceived);
    this.broadcastChannel.addEventListener("message", this.messageReceived);
  }

  /** Handle a message received from another window */
  private messageReceived = (event: MessageEvent) => {
    if (!event.data.type) {
      return;
    }

    const callbacks = this.callbacksByType[event.data.type] ?? new Set();

    if (callbacks.size > 0) {
      const MessageClass = this.typeToMessageClass[event.data.type];

      if (!MessageClass) {
        throw new Error(`No message class registered for message type "${event.data.type}"`);
      }

      const payload = decode(event.data, MessageClass);

      for (const callback of callbacks) {
        callback(payload, event.source);
      }
    }
  };

  /** Stop listening for messages */
  destroy() {
    window.removeEventListener("message", this.messageReceived);
    this.broadcastChannel.removeEventListener("message", this.messageReceived);
  }

  /** Listen for messages from other windows */
  on<T>(MessageClass: CrossWindowMessageType<T>, callback: CrossWindowMessageCallback<T>) {
    const type = MessageClass.type;

    const ExistingMessageClass = this.typeToMessageClass[type];
    if (ExistingMessageClass && ExistingMessageClass !== MessageClass) {
      throw new Error(
        `Message type "${type}" is already registered to class "${ExistingMessageClass.name}", and cannot be registered to class "${MessageClass.name}"`,
      );
    }

    this.typeToMessageClass[type] = MessageClass;

    const callbacks = this.callbacksByType[type] ?? new Set();
    callbacks.add(callback);

    this.callbacksByType[type] = callbacks;

    return () => {
      callbacks.delete(callback);
    };
  }

  /** Send the given message to all windows that share the same broadcast channel */
  send<T>(message: T) {
    this.broadcastChannel.postMessage(encode(message));
  }

  /** Send the given message to the window that opened this window, if any */
  sendToParent<T>(message: T) {
    window.opener?.postMessage(encode(message), "*");
  }
}

/** Encode the given message for sending across windows */
function encode<T>(message: T, MessageClass?: CrossWindowMessageType<T>): { type: string; payload: T } {
  const Class = MessageClass ?? ((message as any).constructor as CrossWindowMessageType<T>);

  if (typeof Class.type !== "string") {
    throw new Error(`Message class ${Class.name} does not have a type property`);
  }

  return { type: Class.type, payload: message };
}

/** Decode the given message received from another window */
function decode<T>(message: { type: string; payload: T }, MessageClass: CrossWindowMessageType<T>): T {
  if (message.type !== MessageClass.type) {
    throw new Error(`Expected message type "${MessageClass.type}" but got "${message.type}"`);
  }

  return new MessageClass(message.payload);
}
