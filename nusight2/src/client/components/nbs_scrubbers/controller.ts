import { ScrubberLoadRequest, ScrubberPauseRequest, ScrubberPlayRequest } from "@proto/message/eye/Scrubber";
import { action } from "mobx";

import { RpcResult } from "../../../shared/messages/generated/rpc_call";
import { RpcNetwork } from "../../hooks/use_rpc_controller";
import { FileDialogController } from "../file_dialog/controller";
import { FileDialogModel } from "../file_dialog/model";

import { NbsScrubbersModel } from "./model";

export class NbsScrubbersController {
  constructor(
    private network: RpcNetwork,
    private model: NbsScrubbersModel,
  ) {}

  toggleLastActiveScrubberPlayback = () => {
    const scrubber = this.model.lastActiveScrubber;
    if (!scrubber) {
      return;
    }

    if (scrubber.playbackState === "playing") {
      this.network.call(new ScrubberPauseRequest({ id: scrubber.id })).then(this.defaultRpcResultHandler);
    } else {
      this.network.call(new ScrubberPlayRequest({ id: scrubber.id })).then(this.defaultRpcResultHandler);
    }
  };

  private defaultRpcResultHandler = <T>(result: RpcResult<T>) => {
    if (!result.ok) {
      if (result.error.isRemoteError()) {
        this.onScrubberError(result.error.message);
      } else {
        result.error.defaultHandler();
      }
    }
  };

  private onScrubberError = (error: string, prefix?: string) => {
    // TODO: add toast messages to NUsight to show errors like this without using alert()
    alert(`${prefix ? prefix + ":\n" : ""}${error}`);
  };
}

export class NbsScrubberDialogController extends FileDialogController {
  constructor(
    private network: RpcNetwork,
    model: FileDialogModel,
  ) {
    super(model);
  }

  @action
  load = (files: string[]) => {
    if (files.length === 0) {
      return;
    }

    this.network.call(new ScrubberLoadRequest({ files })).then((result) => {
      if (!result.ok) {
        if (result.error.isRemoteError() || result.error.isTimeout()) {
          // TODO: add toast messages to NUsight to show errors like this without using alert()
          alert("Scrubber load error:\n" + result.error.message);
        } else {
          result.error.defaultHandler();
        }
      } else {
        // The new scrubber will be added when we get its initial state
      }
    });
  };
}
