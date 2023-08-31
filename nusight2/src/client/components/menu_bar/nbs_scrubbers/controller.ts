import React from "react";
import { action } from "mobx";

import { FilePickerController } from "../../file_picker/controller";

import { NbsScrubbersModel } from "./model";
import { NbsScrubberModel } from "./nbs_scrubber/model";
import { NbsScrubbersNetwork } from "./network";
import { nanosToTimestampObject, percentageToTimestamp } from "./util";

export class NbsScrubbersController {
  model: NbsScrubbersModel;
  network: NbsScrubbersNetwork;
  filePickerController: FilePickerController;

  constructor(model: NbsScrubbersModel, network: NbsScrubbersNetwork) {
    this.model = model;
    this.network = network;
    this.filePickerController = FilePickerController.of({ model: model.filePicker });
  }

  static of(opts: { model: NbsScrubbersModel; network: NbsScrubbersNetwork }) {
    return new NbsScrubbersController(opts.model, opts.network);
  }

  @action
  listFiles = (path: string) => {
    this.network.listScrubberFiles({ directory: path });
  };

  @action
  load = (files: string[]) => {
    if (files.length === 0) {
      return;
    }

    this.network.loadScrubber({ files });
  };

  @action
  close = (scrubber: NbsScrubberModel) => {
    this.network.closeScrubber({ id: scrubber.id });
    this.model.scrubbers.delete(scrubber.id);

    if (this.model.lastActiveScrubber === scrubber) {
      this.model.lastActiveScrubber = undefined;
    }

    if (this.model.scrubbers.size === 0) {
      this.model.showScrubbers = false;
    }
  };

  @action
  togglePlayback = (scrubber: NbsScrubberModel) => {
    if (scrubber.playbackState === "playing") {
      this.network.pauseScrubber({ id: scrubber.id });
    } else {
      this.network.playScrubber({ id: scrubber.id });
    }

    this.model.lastActiveScrubber = scrubber;
  };

  @action
  togglePlaybackForLastActiveScrubber = () => {
    if (this.model.lastActiveScrubber) {
      this.togglePlayback(this.model.lastActiveScrubber);
    }
  };

  @action
  playFaster = (scrubber: NbsScrubberModel) => {
    this.network.setScrubberPlaybackSpeed({ id: scrubber.id, playbackSpeed: scrubber.playbackSpeed + 1 });

    this.model.lastActiveScrubber = scrubber;
  };

  @action
  playSlower = (scrubber: NbsScrubberModel) => {
    this.network.setScrubberPlaybackSpeed({ id: scrubber.id, playbackSpeed: scrubber.playbackSpeed - 1 });

    this.model.lastActiveScrubber = scrubber;
  };

  @action
  toggleRepeat = (scrubber: NbsScrubberModel) => {
    this.network.setScrubberRepeat({ id: scrubber.id, repeat: !scrubber.playbackRepeat });

    this.model.lastActiveScrubber = scrubber;
  };

  @action
  startSeeking = (scrubber: NbsScrubberModel) => {
    scrubber.isSeeking = true;

    this.model.lastActiveScrubber = scrubber;
  };

  @action
  stopSeeking = (scrubber: NbsScrubberModel) => {
    scrubber.isSeeking = false;
  };

  @action
  seek = (scrubber: NbsScrubberModel, event: React.FormEvent<HTMLInputElement>) => {
    // Set the percentPlayed locally for quicker slider feedback.
    // When seeking we ignore timestamp updates from the server.
    scrubber.percentPlayed = Number(event.currentTarget.value);

    const timestamp = percentageToTimestamp(scrubber.percentPlayed, scrubber.start, scrubber.end);

    this.network.seekScrubber({ id: scrubber.id, timestamp: nanosToTimestampObject(timestamp) });

    this.model.lastActiveScrubber = scrubber;
  };
}
