import { computed, observable } from "mobx";

import { NbsScrubber } from "../../../../shared/nbs_scrubber";
import { FilePickerModel } from "../../file_picker/model";

import { NbsScrubberModel } from "./nbs_scrubber/model";

export class NbsScrubbersModel {
  @observable showScrubbers = false;
  @observable scrubbers: Map<NbsScrubber["id"], NbsScrubberModel> = new Map();
  @observable lastActiveScrubber?: NbsScrubberModel;

  @observable filePicker: FilePickerModel = FilePickerModel.of({
    selectedFiles: [],
    currentPath: JSON.parse(localStorage.getItem("nusight:last_picked_path") ?? '"."'),
    recentPaths: JSON.parse(localStorage.getItem("nusight:recently_picked_paths") ?? "[]"),
    quickPaths: [
      { name: "/", path: "/" },
      { name: "Home", path: "~" },
      { name: "NUsight CWD", path: "." },
    ],
  });

  static of() {
    return new NbsScrubbersModel();
  }

  @computed
  get scrubberList() {
    return Array.from(this.scrubbers.values());
  }
}
