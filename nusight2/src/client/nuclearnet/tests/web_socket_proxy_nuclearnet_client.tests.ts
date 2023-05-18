import { createMockInstance } from "../../../shared/base/testing/create_mock_instance";
import { DirectWebSocketClient } from "../direct_web_socket_client";
import { WebSocketProxyNUClearNetClient } from "../web_socket_proxy_nuclearnet_client";
import Mocked = jest.Mocked;

describe("WebSocketProxyNUClearNetClient", () => {
  let mockWebSocket: Mocked<DirectWebSocketClient>;
  let client: WebSocketProxyNUClearNetClient;

  beforeEach(() => {
    mockWebSocket = createMockInstance(DirectWebSocketClient);
    client = new WebSocketProxyNUClearNetClient(mockWebSocket);
  });

  it("forwards connect calls to socket", () => {
    const opts = { name: "bob" };
    client.connect(opts);
    expect(mockWebSocket.connect).toHaveBeenCalled();
  });

  it("forwards disconnect calls to socket", () => {
    const disconnect = client.connect({ name: "bob" });
    disconnect();
    expect(mockWebSocket.disconnect).toHaveBeenCalled();
    // nuclear_disconnect not forwarded, as clients share a single connection.
    expect(mockWebSocket.send).not.toHaveBeenCalledWith("nuclear_disconnect");
  });

  describe("when connected", () => {
    beforeEach(() => {
      client.connect({ name: "bob" });
    });

    it("forwards onJoin add listener requests to socket", () => {
      const cb = jest.fn();
      client.onJoin(cb);
      expect(mockWebSocket.on).toHaveBeenCalledWith("nuclear_join", cb);
    });

    it("forwards onJoin remove listener requests to socket", () => {
      const cb = jest.fn();
      const off = client.onJoin(cb);
      off();
      expect(mockWebSocket.off).toHaveBeenCalledWith("nuclear_join", cb);
    });

    it("forwards onLeave add listener requests to socket", () => {
      const cb = jest.fn();
      client.onLeave(cb);
      expect(mockWebSocket.on).toHaveBeenCalledWith("nuclear_leave", cb);
    });

    it("forwards onLeave remove listener requests to socket", () => {
      const cb = jest.fn();
      const off = client.onLeave(cb);
      off();
      expect(mockWebSocket.off).toHaveBeenCalledWith("nuclear_leave", cb);
    });

    it("forwards generic event add listener requests to socket", () => {
      const cb = jest.fn();
      client.on("foo", cb);
      expect(mockWebSocket.on).toHaveBeenCalledWith("foo", expect.any(Function));
      expect(mockWebSocket.send).toHaveBeenCalledWith("listen", "foo", "0");
    });

    it("increments message id per message", () => {
      const cb = jest.fn();
      client.on("foo", cb);
      client.on("bar", cb);
      client.on("baz", cb);
      expect(mockWebSocket.send).toHaveBeenCalledWith("listen", "foo", "0");
      expect(mockWebSocket.send).toHaveBeenCalledWith("listen", "bar", "1");
      expect(mockWebSocket.send).toHaveBeenCalledWith("listen", "baz", "2");
    });

    it("forwards generic event remove listener requests to socket", () => {
      const cb = jest.fn();
      const off = client.on("foo", cb);
      off();
      expect(mockWebSocket.off).toHaveBeenCalledWith("foo", expect.any(Function));
      expect(mockWebSocket.send).toHaveBeenCalledWith("unlisten", "0");
    });

    it("forwards send calls to socket", () => {
      const opts = {
        type: "foo",
        payload: Buffer.alloc(8),
      };
      client.send(opts);
      expect(mockWebSocket.send).toHaveBeenCalledWith("packet", opts);
    });
  });
});
