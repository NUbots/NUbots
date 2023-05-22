import { NUClearNetPacket, NUClearNetPeer } from "nuclearnet.js";
import path from "path";

import { message, MessageType } from "../../../shared/messages";
import { messageTypeToName } from "../../../shared/messages/type_converters";
import { makeScrubberStatePacket, sampleADefaultState } from "../../nbs_scrubber/tests/test_utils";
import { sampleFileA } from "../../nbs_scrubber/tests/test_utils";
import { samplesDir } from "../../nbs_scrubber/tests/test_utils";
import { scrubberPeerName } from "../../nbs_scrubber/tests/test_utils";
import { tick } from "../../nbs_scrubber/tests/test_utils";
import { hashType } from "../../nuclearnet/hash_type";
import { NUsightSession } from "../session";

import { createMockNUClearNetClient, createMockWebSocket, createPacketFromServer } from "./test_utils";

import Say = message.output.Say;
import ScrubberFileEntry = message.eye.ScrubberFileEntry;
import ScrubberListFilesRequest = message.eye.ScrubberListFilesRequest;
import ScrubberLoadRequest = message.eye.ScrubberLoadRequest;
import ScrubberCloseRequest = message.eye.ScrubberCloseRequest;
import ScrubberSeekRequest = message.eye.ScrubberSeekRequest;
import ScrubberClosed = message.eye.ScrubberClosed;

const sayPacketType = "message.output.Say";

function createSayPacket(opts: { reliable?: boolean } = {}) {
  const payload = Say.encode({ message: "Test" }).finish();

  const packet: NUClearNetPacket = {
    hash: hashType(sayPacketType),
    payload: Buffer.from(payload),
    reliable: opts.reliable ?? false,
    peer: {
      name: "nusight",
      address: "127.0.0.1",
      port: 0,
    },
  };

  return packet;
}

describe("NUsightSession and NUsightSessionClient", () => {
  let nuclearnetClient: ReturnType<typeof createMockNUClearNetClient>["nuclearnetClient"];
  let nuclearnetMockEmit: (event: string, ...args: any[]) => void;

  beforeEach(() => {
    ({ nuclearnetClient, nuclearnetMockEmit } = createMockNUClearNetClient());
  });

  it("forwards outgoing packets from a browser client to NUClearNet", () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // when the client sends a packet to the server, ...
    const packetFromClient = { type: "message.output.Say" };
    mockSocket.emit("packet", packetFromClient);

    // the server should forward the packet to NUClearNet.
    expect(nuclearnetClient.send).toHaveBeenCalledWith(packetFromClient);

    // Clean up
    session.destroy();
  });

  it("listens for incoming NUClearNet packets on behalf of a browser client", () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // and the client starts listening for a given packet type, ...
    mockSocket.emit("listen", sayPacketType, "some-listen-token");

    // when a packet of that type is received from NUClearNet, ...
    const packet = createSayPacket();
    nuclearnetMockEmit(sayPacketType, packet);

    // the packet should be forwarded to the client.
    expect(mockSocket.connection.send).toHaveBeenCalledWith(sayPacketType, packet, expect.any(Function));

    // Ack the send to clear the timeout and end the test
    mockSocket.connection.send.mock.calls[0][2]();

    // Clean up
    session.destroy();
  });

  it("stops listening for incoming NUClearNet packets on request from browser client", () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // if the client starts listening for a given packet type ...
    mockSocket.emit("listen", sayPacketType, "some-listen-token");

    // and then stops listening for that packet type, ...
    mockSocket.emit("unlisten", "some-listen-token");

    const packet = createSayPacket();
    nuclearnetMockEmit("message.output.Say", packet);

    // the packet should not be forwarded to the client since it's no longer listening
    expect(mockSocket.connection.send).not.toHaveBeenCalled();

    // Clean up
    session.destroy();
  });

  it("loads an nbs scrubber on valid scrubber load request from browser client", async () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // if the client starts listening for ScrubberState packets, ...
    mockSocket.emit("listen", "message.eye.ScrubberState", "some-listen-token");

    // and then requests to load a scrubber, ...
    const request = new ScrubberLoadRequest({ files: [sampleFileA], name: scrubberPeerName });
    mockSocket.emitMessage(request);

    await tick();

    const scrubberPeer = {
      name: scrubberPeerName,
      address: "0.0.0.0",
      port: 1,
    };

    // a NUClear join message should be sent to the client to present the scrubber as a "peer", ...
    expect(mockSocket.connection.send).toHaveBeenNthCalledWith(1, "nuclear_join", scrubberPeer);

    const scrubberState = sampleADefaultState();

    // and a ScrubberState packet for the newly loaded scrubber should be sent.
    expect(mockSocket.connection.send).toHaveBeenNthCalledWith(
      2,
      "message.eye.ScrubberState",
      makeScrubberStatePacket({ ...scrubberState, timestamp: scrubberState.start, peer: scrubberPeer }),
      expect.any(Function),
    );

    // Clean up
    session.destroy();
  });

  it("sends loaded scrubbers to all clients in the session", async () => {
    // Given we have a session...
    const session = NUsightSession.of(nuclearnetClient);

    // and a client A connected to the session...
    const mockSocketA = createMockWebSocket();
    session.addClient(mockSocketA.connection);

    // and a client B connected to the session...
    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    // and both clients start listening for ScrubberState packets...
    mockSocketA.emit("listen", "message.eye.ScrubberState", "listen-token-1");
    mockSocketB.emit("listen", "message.eye.ScrubberState", "listen-token-2");

    // when client A requests to load a scrubber...
    const request = new ScrubberLoadRequest({ rpcToken: 1, files: [sampleFileA], name: scrubberPeerName });
    mockSocketA.emitMessage(request);

    await tick();

    const scrubberId = 1;
    const scrubberPeer = { name: scrubberPeerName, address: "0.0.0.0", port: scrubberId };
    const scrubberState = sampleADefaultState(scrubberPeerName);

    // a NUClear join event should be sent to both clients, ...
    expect(mockSocketA.connection.send).toHaveBeenNthCalledWith(1, "nuclear_join", scrubberPeer);
    expect(mockSocketB.connection.send).toHaveBeenNthCalledWith(1, "nuclear_join", scrubberPeer);

    // and a ScrubberState packet should be sent to both clients.
    expect(mockSocketA.connection.send).toHaveBeenNthCalledWith(
      2,
      "message.eye.ScrubberState",
      makeScrubberStatePacket({ ...scrubberState, timestamp: scrubberState.start, peer: scrubberPeer }),
      expect.any(Function),
    );
    expect(mockSocketB.connection.send).toHaveBeenNthCalledWith(
      2,
      "message.eye.ScrubberState",
      makeScrubberStatePacket({ ...scrubberState, timestamp: scrubberState.start, peer: scrubberPeer }),
      expect.any(Function),
    );

    // Clean up
    session.destroy();
  });

  it("sends all currently loaded scrubbers to a new browser client when it connects", async () => {
    // Given we have a session...
    const session = NUsightSession.of(nuclearnetClient);

    // and a client A connected to the session...
    const mockSocketA = createMockWebSocket();
    session.addClient(mockSocketA.connection);

    // when client A loads a scrubber...
    const request = new ScrubberLoadRequest({ rpcToken: 1, files: [sampleFileA], name: scrubberPeerName });
    mockSocketA.emitMessage(request);

    const defaultState = sampleADefaultState(scrubberPeerName);
    const scrubberId = defaultState.id;
    const scrubberPeer = {
      name: scrubberPeerName,
      address: "0.0.0.0",
      port: scrubberId,
    };

    // and seeks to a specific time...
    const seekRequest = new ScrubberSeekRequest({
      rpcToken: 2,
      id: scrubberId,
      timestamp: { seconds: 1149, nanos: 0 },
    });
    mockSocketA.emitMessage(seekRequest);

    await tick();

    // and client B joins...
    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketB.connection);

    // and starts listening for ScrubberState messages, ...
    mockSocketB.emit("listen", "message.eye.ScrubberState", "listen-token-2");

    // a NUClear join event should be sent to client B for the previously loaded scrubber, ...
    expect(mockSocketB.connection.send).toHaveBeenNthCalledWith(1, "nuclear_join", scrubberPeer);

    // along with a corresponding ScrubberState message.
    expect(mockSocketB.connection.send).toHaveBeenNthCalledWith(
      2,
      "message.eye.ScrubberState",
      makeScrubberStatePacket({ ...defaultState, timestamp: { seconds: 1149, nanos: 0 }, peer: scrubberPeer }),
      expect.any(Function),
    );

    // Clean up
    session.destroy();
  });

  it("responds with error on invalid scrubber load request from browser client", async () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // when the client requests to load a scrubber for an invalid file...
    const request = new ScrubberLoadRequest({ rpcToken: 1, files: ["/file/that/does/not/exist.nbs"] });
    mockSocket.emitMessage(request);

    // the scrubber load should fail, with an error response sent back to the client.
    expect(mockSocket.connection.send).toHaveBeenCalledWith(
      "message.eye.ScrubberLoadRequest.Response",
      createPacketFromServer(
        new ScrubberLoadRequest.Response({
          rpcToken: 1,
          ok: false,
          error: "Error: nbs index not found for file: /file/that/does/not/exist.nbs",
        }),
      ),
    );

    // Clean up
    session.destroy();
  });

  it("responds with list of nbs files in a directory on valid request from a browser client", async () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // when the client requests to list scrubber files...
    const request = new ScrubberListFilesRequest({ rpcToken: 1, directory: samplesDir });
    mockSocket.emitMessage(request);

    await callTo(mockSocket.connection.send);

    const response = findAndDecodePacketFromCalls(mockSocket.connection.send, ScrubberListFilesRequest.Response);

    // the request files should be listed in a success response sent back to the client.
    expect(response.payload).toEqual({
      ok: true,
      rpcToken: 1,
      directory: samplesDir,
      entries: [
        {
          type: ScrubberFileEntry.Type.DIRECTORY,
          name: "sample_folder",
          path: path.join(samplesDir, "sample_folder"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
        {
          type: ScrubberFileEntry.Type.FILE,
          name: "sample-000-300.nbs",
          path: path.join(samplesDir, "sample-000-300.nbs"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
        {
          type: ScrubberFileEntry.Type.FILE,
          name: "sample-300-600.nbs",
          path: path.join(samplesDir, "sample-300-600.nbs"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
      ],
    });

    // Clean up
    session.destroy();
  });

  it("responds with error on invalid request to list files from a browser client", async () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // when the client requests to list scrubber files in an invalid directory, ...
    const request = new ScrubberListFilesRequest({ rpcToken: 1, directory: "/invalid/directory" });
    mockSocket.emitMessage(request);

    await callTo(mockSocket.connection.send);

    // ... an error response should be sent back to the client
    expect(mockSocket.connection.send).toHaveBeenCalledWith(
      "message.eye.ScrubberListFilesRequest.Response",
      createPacketFromServer(
        new ScrubberListFilesRequest.Response({
          rpcToken: 1,
          ok: false,
          error: "Error: ENOENT: no such file or directory, scandir '/invalid/directory'",
        }),
      ),
    );

    // Clean up
    session.destroy();
  });

  it("handles scrubber seek requests from a browser client", async () => {
    // Given we have a session with a mock client connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocket = createMockWebSocket();
    session.addClient(mockSocket.connection);

    // when the client listen to a packet type, ...
    // (this tests that types registered before the scrubber is loaded are handled correctly)
    mockSocket.emit("listen", "message.Ping", "listen-token-ping");

    // and it loads a scrubber, ...
    const loadRequest = new ScrubberLoadRequest({ rpcToken: 1, files: [sampleFileA] });
    mockSocket.emitMessage(loadRequest);

    // (ack the packet sent on scrubber load)
    mockSocket.connection.send.mock.calls[1][2](0, 0);
    await tick();

    // and then listens to another packet type, ...
    // (this tests that types registered after the scrubber is loaded are handled correctly)
    mockSocket.emit("listen", "message.Pong", "listen-token-pong");

    // (ack the packet sent from the Pong listen)
    mockSocket.connection.send.mock.calls[3][2](0, 0);
    await tick();

    // when it requests to seek to a specific time ...
    const updateRequest = new ScrubberSeekRequest({
      rpcToken: 2,
      id: 1,
      timestamp: { seconds: 1149, nanos: 0 },
    });
    mockSocket.emitMessage(updateRequest);

    await tick();

    // ... the scrubber should seek to the requested time and respond to the client with packets
    // at the seeked timestamp matching the packet types the client is currently listening for.
    // The `connection.send()` mock contains all the calls to send, with the arguments
    // of each call, so we match against a snapshot to make sure the right packets
    // were sent in the right order.
    expect(mockSocket.connection.send).toMatchSnapshot();

    // Clean up
    session.destroy();
  });

  it("closes a loaded scrubber on request from browser client", async () => {
    // Given we have a session with a mock clients A and B connected, ...
    const session = NUsightSession.of(nuclearnetClient);
    const mockSocketA = createMockWebSocket();
    const mockSocketB = createMockWebSocket();
    session.addClient(mockSocketA.connection);
    session.addClient(mockSocketB.connection);

    // and client A loads a scrubber, ...
    const loadRequest = new ScrubberLoadRequest({ rpcToken: 1, files: [sampleFileA] });
    mockSocketA.emitMessage(loadRequest);

    const scrubberId = 1;

    // when client A closes the scrubber ...
    const stopRequest = new ScrubberCloseRequest({ rpcToken: 2, id: scrubberId });
    mockSocketA.emitMessage(stopRequest);

    await tick();

    // ... a NUClear leave event should be sent to both clients, ...
    const expectedLeavePeer = { name: "sample-000-300.nbs", address: "0.0.0.0", port: scrubberId };
    expect(mockSocketA.connection.send).toHaveBeenCalledWith("nuclear_leave", expectedLeavePeer);
    expect(mockSocketB.connection.send).toHaveBeenCalledWith("nuclear_leave", expectedLeavePeer);

    // ... a ScrubberClosed event should be sent to both clients, ...
    const expectedScrubberClosed = createPacketFromServer(new ScrubberClosed({ id: scrubberId }));
    expect(mockSocketA.connection.send).toHaveBeenCalledWith("message.eye.ScrubberClosed", expectedScrubberClosed);
    expect(mockSocketB.connection.send).toHaveBeenCalledWith("message.eye.ScrubberClosed", expectedScrubberClosed);

    // ... as well as a success response to the client that requested the scrubber to be closed.
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(
      "message.eye.ScrubberCloseRequest.Response",
      createPacketFromServer(new ScrubberCloseRequest.Response({ rpcToken: 2, ok: true })),
    );

    // Attempting to seek the closed scrubber...
    const updateRequest = new ScrubberSeekRequest({
      rpcToken: 3,
      id: scrubberId,
      timestamp: { seconds: 1100, nanos: 0 },
    });
    mockSocketA.emitMessage(updateRequest);

    await tick();

    // ... should result in an error.
    expect(mockSocketA.connection.send).toHaveBeenCalledWith(
      "message.eye.ScrubberSeekRequest.Response",
      createPacketFromServer(
        new ScrubberSeekRequest.Response({
          rpcToken: 3,
          ok: false,
          error: `scrubber ${scrubberId} not found for update (seek)`,
        }),
      ),
    );

    // Clean up
    session.destroy();
  });
});

/**
 * Used to wait for an asynchronous call to a mock function that has an `onCalled()` notifier.
 * A timeout can be set for how long to wait before giving up.
 */
function callTo<T extends { onCalled(callback: () => void): void }>(
  mockWithNotifier: T,
  opts: { timeout?: number } = {},
) {
  return new Promise<void>((resolve, reject) => {
    const timeout = opts.timeout ?? 250;

    // Reject if the mock is not called within the timeout period
    const waitTimeout = setTimeout(() => {
      reject("Timed out waiting for mock to be called");
    }, timeout);

    // Wait for the mock to be called
    mockWithNotifier.onCalled(() => {
      // Clear the wait timeout
      clearTimeout(waitTimeout);

      // Resolve the promise
      resolve();
    });
  });
}

/** Find the first packet of the given type in the mock function's call arguments, and decode it */
function findAndDecodePacketFromCalls<T>(
  mockFn: jest.Mock,
  packetType: MessageType<T>,
): {
  hash: Buffer;
  reliable: boolean;
  payload: T;
  peer: NUClearNetPeer;
} {
  const packetTypeName = messageTypeToName(packetType);
  const response = mockFn.mock.calls.find((call) => call[0] === packetTypeName)?.[1];
  const payload = packetType.decode(response.payload);
  return { ...response, payload };
}
