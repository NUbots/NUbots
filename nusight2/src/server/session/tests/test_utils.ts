import { NUClearNetPacket, NUClearNetSend } from "nuclearnet.js";

import { createMockEventEmitter } from "../../../shared/base/testing/create_mock_event_emitter";
import { MessageType } from "../../../shared/messages";
import { messageTypeToName } from "../../../shared/messages/type_converters";
import { NUClearNetClient } from "../../../shared/nuclearnet/nuclearnet_client";
import { FakeNUClearNetClient } from "../../nuclearnet/fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../../nuclearnet/fake_nuclearnet_server";
import { hashType } from "../../nuclearnet/hash_type";
import { ClientConnection } from "../../web_socket/client_connection";

/**
 * Create a mock web socket connection that can simulate emits
 * which trigger the socket's `on()` event listener
 */
export function createMockWebSocket(): {
  connection: ClientConnection & {
    send: MockWithNotifier<void, [string, ...any[]], () => void>;
  };
  emit: (event: string, ...args: any[]) => void;
  emitMessage: (message: any) => void;
} {
  const mockEmitter = createMockEventEmitter();

  return {
    connection: {
      onDisconnect: jest.fn(),
      send: createMockFnWithNotifier(),
      on: mockEmitter.on,
    },
    emit: mockEmitter.emit,
    emitMessage: (message: any) => {
      const messageType = message.constructor as MessageType<any>;
      const type = messageTypeToName(messageType);
      const payload = messageType.encode(message).finish() as Buffer;
      const packet: NUClearNetSend = { payload, type, reliable: true, target: "nusight" };
      mockEmitter.emit("packet", packet);
    },
  };
}

/**
 * Create a fake NUClearNetClient connected to a fake server with mock implementations
 * of the `on()` and `send()` methods
 */
export function createMockNUClearNetClient(): {
  nuclearnetClient: NUClearNetClient & {
    on: jest.Mock<() => void, [event: string, callback: (...args: any[]) => void]>;
    send: jest.Mock<void, [event: string, ...args: any[]]>;
  };
  nuclearnetMockEmit: (event: string, ...args: any[]) => void;
} {
  const nuclearnetServer = new FakeNUClearNetServer();
  const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer);

  const mockEmitter = createMockEventEmitter();
  nuclearnetClient.on = mockEmitter.on;

  nuclearnetClient.send = jest.fn();

  return { nuclearnetClient: nuclearnetClient as any, nuclearnetMockEmit: mockEmitter.emit };
}

/**
 * A mock function with a `.onCalled()` method that can be used to attach
 * a callback for notification when the function is called
 */
type MockWithNotifier<MockReturn, MockArgs extends any[], Callback extends CallableFunction> = jest.Mock<
  MockReturn,
  MockArgs
> & {
  onCalled: (callback: Callback) => void;
};

/**
 * Creates a mock function with a `onCalled()` method that can be used to attach
 * a callback for notification when the function is called
 */
function createMockFnWithNotifier<
  MockReturn = any,
  MockArgs extends any[] = any[],
  OnCalledListener extends CallableFunction = () => void,
>(mockImplementation?: (...args: MockArgs) => MockReturn): MockWithNotifier<MockReturn, MockArgs, OnCalledListener> {
  const callbacks = new Set<OnCalledListener>();

  const onCalled = (callback: OnCalledListener) => {
    callbacks.add(callback);
  };

  const mock = jest.fn().mockImplementation((...args: MockArgs) => {
    mockImplementation?.(...args);

    // Call all the notifier's callbacks on the next tick
    setImmediate(() => {
      for (const callback of callbacks) {
        callback();
      }
    });
  });

  return Object.assign(mock, { onCalled });
}

/** Create a packet from the NUsight server with the given message type */
export function createPacketFromServer(message: any): NUClearNetPacket {
  const messageType = message.constructor as MessageType<any>;
  const typeName = messageTypeToName(messageType);
  const hash = hashType(typeName);
  const payload = messageType.encode(message).finish() as Buffer;

  return {
    hash,
    payload,
    reliable: true,
    peer: {
      name: "nusight",
      address: "0.0.0.0",
      port: 0,
    },
  };
}

/** Create a packet from NUClearNet server with the given message type */
export function createPacketFromNUClearNet(message: any, opts: { reliable?: boolean } = {}) {
  const messageType = message.constructor as MessageType<any>;
  const typeName = messageTypeToName(messageType);
  const hash = hashType(typeName);
  const payload = messageType.encode(message).finish() as Buffer;

  const packet: NUClearNetPacket = {
    hash,
    payload,
    reliable: opts.reliable ?? false,
    peer: {
      name: "nuclearnet_client",
      address: "127.0.0.1",
      port: 0,
    },
  };

  return packet;
}
