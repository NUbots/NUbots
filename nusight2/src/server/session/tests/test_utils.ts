import { vi } from "vitest";
import { NUClearNetPacket, NUClearNetPeer, NUClearNetSend } from "nuclearnet.js";

import { AwaitableMock, createAwaitableMock } from "../../../shared/base/testing/awaitable_mock";
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
    send: AwaitableMock<void, [string, ...any[]]>;
  };
  emit: (event: string, ...args: any[]) => void;
  emitMessage: (message: any) => void;
} {
  const mockEmitter = createMockEventEmitter();

  return {
    connection: {
      onDisconnect: vi.fn(),
      on: mockEmitter.on,
      send: createAwaitableMock<void, [string, ...any[]]>(),
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

  nuclearnetClient.send = vi.fn();

  return { nuclearnetClient: nuclearnetClient as any, nuclearnetMockEmit: mockEmitter.emit };
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

/** Find the first packet of the given type in the mock function's call arguments */
export function findPacketFromCalls(
  mockFn: jest.Mock,
  packetType: MessageType<any>,
  nthMatchingPacket: number = 1,
): NUClearNetPacket | undefined {
  const packetTypeName = messageTypeToName(packetType);

  let matchedCount = 0;
  const matchedCall = mockFn.mock.calls.find((callArgs) => {
    if (callArgs[0] === packetTypeName) {
      matchedCount++;
      return matchedCount === nthMatchingPacket;
    }
    return false;
  });

  return matchedCall?.[1];
}

/** Find the first packet of the given type in the mock function's call arguments, and decode it */
export function findAndDecodePacketFromCalls<T>(
  mockFn: jest.Mock,
  packetType: MessageType<T>,
  nthMatchingPacket: number = 1,
):
  | {
      hash: Buffer;
      reliable: boolean;
      payload: T;
      peer: NUClearNetPeer;
    }
  | undefined {
  const packet = findPacketFromCalls(mockFn, packetType, nthMatchingPacket);

  if (packet) {
    const payload = packetType.decode(packet.payload);
    return { ...packet, payload };
  }
}
