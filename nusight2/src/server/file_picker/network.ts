import { FileEntry_TypeEnum, ListFilesRequest } from "@proto/message/eye/File";

import { NUsightSession } from "../session/session";

import { listFiles } from "./list_files";

export class FilePickerNetwork {
  private cleanUp?: () => void;

  constructor(session: NUsightSession) {
    const network = session.network;
    this.cleanUp = network.onClientRpc(ListFilesRequest, this.onListFiles);
  }

  /** Destroy the network by removing all event listeners */
  public destroy() {
    if (this.cleanUp) {
      this.cleanUp();
      this.cleanUp = undefined;
    }
  }

  /** Handle a request from the client to list local NBS files (for loading a scrubber, etc) */
  private onListFiles = async (request: ListFilesRequest) => {
    const { entries, directory } = await listFiles(request.directory, request.type);

    return new ListFilesRequest.Response({
      rpc: { ok: true, token: request.rpc?.token },
      directory,
      entries: entries.map((entry) => {
        return {
          type: entry.type === "directory" ? FileEntry_TypeEnum.DIRECTORY : FileEntry_TypeEnum.FILE,
          name: entry.name,
          path: entry.path,
          size: BigInt(entry.size),
          dateModified: {
            seconds: BigInt(Math.floor(entry.dateModified.getTime() / 1000)),
            nanos: (entry.dateModified.getTime() % 1000) * 1e6,
          },
        };
      }),
    });
  };
}
