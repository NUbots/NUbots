import { action, computed, observable, reaction } from "mobx";

import { NbsScrubbersModel } from "./model";

export class NbsScrubbersViewModel {
  @observable areScrubbersVisible = false;
  @observable isFileDialogVisible = false;

  private previousScrubberCount: number;

  constructor(private model: NbsScrubbersModel) {
    this.previousScrubberCount = this.model.scrubbers.size;

    // Set up reactions to changes in the number of open scrubbers
    reaction(
      () => this.model.scrubbers.size,
      (size) => {
        // If all scrubbers removed, hide their visibility
        if (size === 0) {
          this.areScrubbersVisible = false;
        }

        // Show scrubbers when the first is added
        if (this.previousScrubberCount === 0) {
          this.areScrubbersVisible = true;
        }

        this.previousScrubberCount = size;
      },
    );
  }

  @action.bound
  toggleScrubbers() {
    this.areScrubbersVisible = !this.areScrubbersVisible;
  }

  @action.bound
  showFileDialog() {
    this.isFileDialogVisible = true;
  }

  @action.bound
  hideFileDialog() {
    this.isFileDialogVisible = false;
  }

  @computed
  get scrubbers() {
    return this.model.scrubbers;
  }

  @computed
  get scrubberList() {
    return Array.from(this.scrubbers.values());
  }
}
