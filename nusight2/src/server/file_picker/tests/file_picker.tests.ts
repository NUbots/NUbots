import path from "path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";

import { message } from "../../../shared/messages";
import { samplesDir } from "../../nbs_scrubber/tests/test_utils";
import { NUsightSession } from "../../session/session";
import { createMockNUClearNetClient, findAndDecodePacketFromCalls } from "../../session/tests/test_utils";
import { createMockWebSocket } from "../../session/tests/test_utils";
import { createPacketFromServer } from "../../session/tests/test_utils";
import { createFilePicker } from "../create";

import FileEntry = message.eye.FileEntry;
import FilesRequestType = message.eye.FilesRequestType;
import ListFilesRequest = message.eye.ListFilesRequest;

describe("File Picker", () => {
  let session: NUsightSession;

  beforeEach(() => {
    // Setup a session with the file picker network and a socket to send messages through
    const { nuclearnetClient } = createMockNUClearNetClient();
    session = new NUsightSession(nuclearnetClient, [createFilePicker]);
  });

  afterEach(() => {
    session.destroy();
  });

  it("responds with list of directories in a directory on valid request from a browser client", async () => {
    const socket = createMockWebSocket();
    session.addClient(socket.connection);

    // when the client requests to list directories...
    const request = new ListFilesRequest({
      rpc: { token: 1 },
      directory: samplesDir,
      type: FilesRequestType.DIRECTORY,
    });
    socket.emitMessage(request);

    await socket.connection.send.waitForCall();

    const response = findAndDecodePacketFromCalls(socket.connection.send, ListFilesRequest.Response);

    // the requested directories should be listed in a response sent back to the client.
    expect(response?.payload).toEqual({
      rpc: { ok: true, token: 1 },
      directory: samplesDir,
      entries: [
        {
          type: FileEntry.Type.DIRECTORY,
          name: "sample_folder",
          path: path.join(samplesDir, "sample_folder"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
      ],
    });
  });

  it("responds with list of nbs files in a directory on valid request from a browser client", async () => {
    const socket = createMockWebSocket();
    session.addClient(socket.connection);

    // when the client requests to list NBS files...
    const request = new ListFilesRequest({ rpc: { token: 1 }, directory: samplesDir, type: FilesRequestType.NBS });
    socket.emitMessage(request);

    await socket.connection.send.waitForCall();

    const response = findAndDecodePacketFromCalls(socket.connection.send, ListFilesRequest.Response);

    // the requested files should be listed in a response sent back to the client.
    expect(response?.payload).toEqual({
      rpc: { ok: true, token: 1 },
      directory: samplesDir,
      entries: [
        {
          type: FileEntry.Type.DIRECTORY,
          name: "sample_folder",
          path: path.join(samplesDir, "sample_folder"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
        {
          type: FileEntry.Type.FILE,
          name: "sample-000-300.nbs",
          path: path.join(samplesDir, "sample-000-300.nbs"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
        {
          type: FileEntry.Type.FILE,
          name: "sample-300-600.nbs",
          path: path.join(samplesDir, "sample-300-600.nbs"),
          size: expect.any(Number),
          dateModified: { seconds: expect.any(Number) },
        },
      ],
    });
  });

  it("responds with error on invalid request to list files from a browser client", async () => {
    const socket = createMockWebSocket();
    session.addClient(socket.connection);

    // when the client requests to list files in an invalid directory, ...
    const request = new ListFilesRequest({ rpc: { token: 1 }, directory: "/invalid/directory" });
    socket.emitMessage(request);

    await socket.connection.send.waitForCall();

    // ... an error response should be sent back to the client
    expect(socket.connection.send).toHaveBeenCalledWith(
      "message.eye.ListFilesRequest.Response",
      createPacketFromServer(
        new ListFilesRequest.Response({
          rpc: { token: 1, ok: false, error: "Error: ENOENT: no such file or directory, scandir '/invalid/directory'" },
        }),
      ),
    );
  });
});
