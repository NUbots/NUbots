import { useEffect, useMemo } from "react";
import { action } from "mobx";

import { message } from "../../../shared/messages";
import { TimestampObject } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { useNUsightNetwork } from "../app/context";

import { FileDialogModel } from "./model";

export type FilesRequestType = "unknown" | "directory" | "nbs";

const FilesRequestTypeToEnum: Record<FilesRequestType, message.eye.FilesRequestType> = {
  unknown: message.eye.FilesRequestType.UNKNOWN,
  directory: message.eye.FilesRequestType.DIRECTORY,
  nbs: message.eye.FilesRequestType.NBS,
};

export function useFileDialogNetwork(model: FileDialogModel) {
  // Create the file dialog network
  const nusightNetwork = useNUsightNetwork();
  const network = useMemo(() => FileDialogNetwork.of(nusightNetwork, model), [nusightNetwork, model]);

  // Disconnect network when the network changes or the component is destroyed
  useEffect(() => () => network.off(), [network]);

  return network;
}

export class FileDialogNetwork {
  constructor(private network: Network, private model: FileDialogModel) {}

  static of(nusightNetwork: NUsightNetwork, model: FileDialogModel): FileDialogNetwork {
    const network = Network.of(nusightNetwork);
    return new FileDialogNetwork(network, model);
  }

  off() {
    this.network.off();
  }

  async listFiles({ directory, type }: { directory: string; type: FilesRequestType }) {
    const request = new message.eye.ListFilesRequest({ directory, type: FilesRequestTypeToEnum[type] });
    const result = await this.network.call(request);

    if (result.ok) {
      this.onListFilesSuccess(result.data.response);
    } else {
      if (result.error.isRemoteError()) {
        this.onListFilesError(result.error.message);
      } else {
        result.error.defaultHandler();
      }
    }
  }

  @action
  private onListFilesSuccess = (response: message.eye.ListFilesRequest.Response) => {
    this.model.error = "";
    this.model.currentPath = response.directory!;
    this.model.currentPathEntries = (response.entries ?? []).map((entry) => {
      return {
        type: entry.type === message.eye.FileEntry.Type.DIRECTORY ? "directory" : "file",
        name: entry.name!,
        path: entry.path!,
        size: entry.size!,
        dateModified: new Date(TimestampObject.toMillis(entry.dateModified)),
      };
    });
  };

  @action
  private onListFilesError = (error: string) => {
    this.model.error = error;
  };
}
