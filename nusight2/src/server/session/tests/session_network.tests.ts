import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { NUClearNetPacket, NUClearNetSend } from "nuclearnet.js";

import { message } from "../../../shared/messages";
import { messageTypeToName } from "../../../shared/messages/type_converters";
import { NUsightSession } from "../session";
import { NUsightSessionNetwork } from "../session_network";

import {
  createMockNUClearNetClient,
  createMockWebSocket,
  createPacketFromNUClearNet,
  createPacketFromServer,
} from "./test_utils";

import Test = message.support.nusight.Test;
import ScrubberLoadRequest = message.eye.ScrubberLoadRequest;

const testPacketType = messageTypeToName(Test);

describe("NUsightSessionNetwork", () => {
  let nuclearnetClient: ReturnType<typeof createMockNUClearNetClient>["nuclearnetClient"];
  let session: NUsightSession;
  let network: NUsightSessionNetwork;
  let nuclearnetMockEmit: (event: string, ...args: any[]) => void;

  beforeEach(() => {
    ({ nuclearnetClient, nuclearnetMockEmit } = createMockNUClearNetClient());
    session = new NUsightSession(nuclearnetClient, []);
    network = new NUsightSessionNetwork(session);
  });

  afterEach(() => {
    session.destroy();
  });

  it("emit() sends to all clients in the session when target is `undefined`", () => {
    const mockSocketA = createMockWebSocket();
    session.addClient(mockSocketA.connection);

    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    const packet = new Test({ message: "hello world" });
    network.emit(packet);

    const expectedPacket = createPacketFromServer(packet);
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
    expect(mockSocketB.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
  });

  it("emit() sends to all clients in the session when target is `nusight`", () => {
    const mockSocketA = createMockWebSocket();
    session.addClient(mockSocketA.connection);

    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    const packet = new Test({ message: "hello world" });
    network.emit(packet, { target: "nusight" });

    const expectedPacket = createPacketFromServer(packet);
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
    expect(mockSocketB.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
  });

  it("emit() throws if target refers to a client that doesn't exist in the session", () => {
    const unknownClientId = 999;

    expect(() => {
      network.emit(new Test({ message: "hello world" }), { target: `nusight#${unknownClientId}` });
    }).toThrowError("no such client");
  });

  it("emit() sends to a specific client when client ID is specified in target", () => {
    const mockSocketA = createMockWebSocket();
    const clientA = session.addClient(mockSocketA.connection);

    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    const packet = new Test({ message: "hello world" });
    network.emit(packet, { target: `nusight#${clientA.id}` });

    const expectedPacket = createPacketFromServer(packet);
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);

    expect(mockSocketB.connection.send).not.toHaveBeenCalled();
  });

  it("emit() sends to all clients and NUClearNet with target is `*`", () => {
    const mockSocketA = createMockWebSocket();
    session.addClient(mockSocketA.connection);

    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    const packet = new Test({ message: "hello world" });
    network.emit(packet, { target: "*" });

    const expectedPacket = createPacketFromServer(packet);
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
    expect(mockSocketB.connection.send).toHaveBeenCalledWith(testPacketType, expectedPacket);
    expect(nuclearnetClient.send).toHaveBeenCalledWith({
      type: expectedPacket.hash,
      payload: expectedPacket.payload,
      reliable: expectedPacket.reliable,
    });
  });

  it("emit() sends to NUClearNet when target is the empty string", () => {
    const packet = new Test({ message: "hello world" });
    network.emit(packet, { target: "" });

    const expectedPacket = createPacketFromServer(packet);
    expect(nuclearnetClient.send).toHaveBeenCalledWith({
      type: expectedPacket.hash,
      payload: expectedPacket.payload,
      target: "",
      reliable: expectedPacket.reliable,
    });
  });

  it("emit() sends to NUClearNet when target is a string without a special meaning", () => {
    const packet = new Test({ message: "hello world" });
    network.emit(packet, { target: "nuclearnet_client_4" });

    const expectedPacket = createPacketFromServer(packet);
    expect(nuclearnetClient.send).toHaveBeenCalledWith({
      type: expectedPacket.hash,
      payload: expectedPacket.payload,
      target: "nuclearnet_client_4",
      reliable: expectedPacket.reliable,
    });
  });

  it("onNUClearMessage() listens for incoming messages from NUClearNet", () => {
    // Add a NUClearNet listener and check that it returns a function to remove the listener
    const onTest = vi.fn();
    const off = network.onNUClearMessage({ type: Test }, onTest);
    expect(typeof off).toBe("function");

    // Emit a packet from NUClearNet and make sure the listener is called
    const packet: NUClearNetPacket = createPacketFromNUClearNet(new Test({ message: "hello world" }));
    nuclearnetMockEmit(testPacketType, packet);
    expect(onTest).toHaveBeenCalledWith(packet.peer, { message: "hello world" });

    // Remove the listener
    off();

    // Emit a packet from NUClearNet again and make sure the listener
    // that was removed is not called again
    nuclearnetMockEmit(testPacketType, packet);
    expect(onTest).toHaveBeenCalledTimes(1);
  });

  it("onClientMessage() listens for incoming messages from a client in the session", () => {
    // Add a client listener and check that it returns a function to remove the listener
    const onTest = vi.fn();
    const off = network.onClientMessage({ type: Test }, onTest);
    expect(typeof off).toBe("function");

    // Create a client and add it to the session
    const mockSocket = createMockWebSocket();
    const client = session.addClient(mockSocket.connection);

    // Emit a packet from the client and make sure the listener is called
    const clientSend: NUClearNetSend = {
      type: testPacketType,
      payload: Test.encode({ message: "hello world" }).finish() as Buffer,
      // Target the packet to the NUsight server, so it's not sent to NUclearNet
      target: "nusight",
    };
    mockSocket.emit("packet", clientSend);
    expect(onTest).toHaveBeenCalledWith(client, { message: "hello world" });

    // Remove the listener
    off();

    // Emit a packet from the client again and make sure the listener
    // that was removed is not called again
    mockSocket.emit("packet", clientSend);
    expect(onTest).toHaveBeenCalledTimes(1);
  });

  it("onClientRpc() listens for incoming RPC requests from a client and sends a response on success", async () => {
    // Create a handler for the RPC request
    const onScrubberLoadRequest = vi.fn().mockImplementation((request: ScrubberLoadRequest) => {
      return new ScrubberLoadRequest.Response({
        rpc: {
          token: request.rpc?.token,
          ok: true,
        },
      });
    });

    // Register the handler and check that it returns a function to remove the listener
    const off = network.onClientRpc({ type: ScrubberLoadRequest }, onScrubberLoadRequest);
    expect(typeof off).toBe("function");

    // Create a client and add it to the session
    const mockSocket = createMockWebSocket();
    const client = session.addClient(mockSocket.connection);

    const requestTypeName = messageTypeToName(ScrubberLoadRequest);
    const request = {
      rpc: { token: 1 },
      files: ["a.nbs", "b.nbs"],
    };

    // Emit a RPC request from the client and make sure the handler is called
    const clientSend: NUClearNetSend = {
      type: requestTypeName,
      payload: ScrubberLoadRequest.encode(request).finish() as Buffer,
      // Target the packet to the NUsight server, so it's not sent to NUclearNet
      target: "nusight",
    };
    mockSocket.emit("packet", clientSend);
    expect(onScrubberLoadRequest).toHaveBeenCalledWith(request, client);

    await tick();

    // Check that the response was sent to the client
    const expectedResponseType = messageTypeToName(ScrubberLoadRequest.Response);
    const expectedResponse = createPacketFromServer(
      new ScrubberLoadRequest.Response({
        rpc: {
          token: request.rpc.token,
          ok: true,
        },
      }),
    );
    expect(mockSocket.connection.send).toHaveBeenCalledWith(expectedResponseType, expectedResponse);

    // Remove the RPC listener
    off();

    // Emit a packet from the client again and make sure the listener
    // that was removed is not called again
    mockSocket.emit("packet", clientSend);
    expect(onScrubberLoadRequest).toHaveBeenCalledTimes(1);
  });

  it("onClientRpc() listens for incoming RPC requests from a client and sends an error response on error", async () => {
    // Create a handler for the RPC request that throws an error
    const onScrubberLoadRequest = vi.fn().mockImplementation(() => {
      throw new Error("Something went wrong");
    });

    // Register the handler and check that it returns a function to remove the listener
    network.onClientRpc({ type: ScrubberLoadRequest }, onScrubberLoadRequest);

    // Create a client and add it to the session
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    const requestTypeName = messageTypeToName(ScrubberLoadRequest);
    const request = {
      rpc: { token: 1 },
      files: ["a.nbs", "b.nbs"],
    };

    // Emit a RPC request from the client and make sure the handler is called
    const clientSend: NUClearNetSend = {
      type: requestTypeName,
      payload: ScrubberLoadRequest.encode(request).finish() as Buffer,
      // Target the packet to the NUsight server, so it's not sent to NUclearNet
      target: "nusight",
    };
    mockSocket.emit("packet", clientSend);

    await tick();

    // Check that the response was sent to the client
    const expectedResponseType = messageTypeToName(ScrubberLoadRequest.Response);
    const expectedResponse = createPacketFromServer(
      new ScrubberLoadRequest.Response({
        rpc: {
          token: request.rpc.token,
          ok: false,
          error: "Something went wrong",
        },
      }),
    );
    expect(mockSocket.connection.send).toHaveBeenCalledWith(expectedResponseType, expectedResponse);
  });
});

function tick() {
  return new Promise((resolve) => {
    setImmediate(resolve);
  });
}
