import { NUClearNetPacket, NUClearNetPeer, NUClearNetSend } from "nuclearnet.js";
import { Mock, vi } from "vitest";

import { AwaitableMock, createAwaitableMock } from "../../../shared/base/testing/awaitable_mock";
import { createMockEventEmitter } from "../../../shared/base/testing/create_mock_event_emitter";
import { MessageType } from "../../../shared/messages/types";
import { hashType } from "../../../shared/nuclearnet/hash_type";
import { NUClearNetClient } from "../../../shared/nuclearnet/nuclearnet_client";
import { FakeNUClearNetClient } from "../../nuclearnet/fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../../nuclearnet/fake_nuclearnet_server";
import { ClientConnection } from "../../web_socket/client_connection";

/**
 * Create a mock web socket connection that can simulate emits
 * which trigger the socket's `on()` event listener
 */
export function createMockWebSocket(): {
  connection: ClientConnection & {
    send: AwaitableMock<(event: string, ...args: any[]) => void>;
  };
  emit: (event: string, ...args: any[]) => void;
  emitMessage: (message: any) => void;
} {
  const mockEmitter = createMockEventEmitter();

  return {
    connection: {
      onDisconnect: vi.fn(),
      on: mockEmitter.on,
      send: createAwaitableMock<(event: string, ...args: any[]) => void>(),
    },
    emit: mockEmitter.emit,
    emitMessage: (message: any) => {
      const messageType = message.constructor as MessageType<any>;
      const type = messageType.typeName;
      const payload = Buffer.from(message.toBinary());
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
    on: Mock<(event: string, callback: (...args: any[]) => void) => () => void>;
    send: Mock<(event: string, ...args: any[]) => void>;
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
  const typeName = messageType.typeName;
  const hash = hashType(typeName);
  const payload = Buffer.from(message.toBinary());

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
  const typeName = messageType.typeName;
  const hash = hashType(typeName);
  const payload = Buffer.from(message.toBinary());

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
  mockFn: Mock,
  packetType: MessageType<any>,
  nthMatchingPacket: number = 1,
): NUClearNetPacket | undefined {
  const packetTypeName = packetType.typeName;

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
  mockFn: Mock,
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
    const payload = packetType.fromBinary(packet.payload as Uint8Array);
    return { ...packet, payload };
  }
}
