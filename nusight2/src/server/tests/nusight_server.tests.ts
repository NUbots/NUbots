import { beforeEach, describe, expect, it } from "vitest";

import { MockEventHandler } from "../../shared/base/testing/create_mock_event_handler";
import { createMockEventHandler } from "../../shared/base/testing/create_mock_event_handler";
import { createMockInstance } from "../../shared/base/testing/create_mock_instance";
import { FakeNUClearNetClient } from "../nuclearnet/fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../nuclearnet/fake_nuclearnet_server";
import { NUsightServer } from "../nusight_server";
import { WebSocket, WebSocketServer } from "../web_socket/web_socket_server";

describe("NUsightServer", () => {
  let webSocketServer: jest.Mocked<WebSocketServer>;
  let onClientConnection: MockEventHandler<[WebSocket]>;
  let nuclearnetServer: FakeNUClearNetServer;
  let nuclearnetClient: FakeNUClearNetClient;

  beforeEach(() => {
    webSocketServer = createMockInstance(WebSocketServer);
    onClientConnection = createMockEventHandler<[WebSocket]>();
    webSocketServer.onConnection = onClientConnection;
    nuclearnetServer = new FakeNUClearNetServer();
    nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer);
    nuclearnetClient.connect({ name: "bob" });
    new NUsightServer(webSocketServer, nuclearnetClient, {
      name: "test",
      address: "10.1.255.255",
    });
  });

  it("listens to new connections", () => {
    expect(webSocketServer.onConnection).toHaveBeenCalledTimes(1);
  });

  it("forwards NUClearNet network join events to socket", () => {
    const webSocket = createMockInstance(WebSocket);
    onClientConnection.mockEvent(webSocket);

    const alice = new FakeNUClearNetClient(nuclearnetServer);
    alice.connect({ name: "alice" });

    expect(webSocket.send).toHaveBeenLastCalledWith(
      "nuclear_join",
      expect.objectContaining({
        name: "alice",
      }),
    );
  });

  it("forwards NUClearNet network leave events to socket", () => {
    const webSocket = createMockInstance(WebSocket);
    onClientConnection.mockEvent(webSocket);

    const alice = new FakeNUClearNetClient(nuclearnetServer);
    const disconnect = alice.connect({ name: "alice" });
    disconnect();

    expect(webSocket.send).toHaveBeenLastCalledWith(
      "nuclear_leave",
      expect.objectContaining({
        name: "alice",
      }),
    );
  });
});
