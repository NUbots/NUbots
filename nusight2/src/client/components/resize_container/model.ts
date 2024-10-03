import { fitChildren } from "./fit_children";
import { moveHandle } from "./move_handle";
import { ResizePanelProps, ResizePanelState } from "./resize_panel";

const STORAGE_KEY_PREFIX = "nusight:resize_container_sizes:";

/** Load the state of child panels from a key */
function loadState(key: string, childCount: number) {
  const storageItem = localStorage.getItem(STORAGE_KEY_PREFIX + key);
  if (!storageItem) {
    return undefined;
  }

  const savedValues = JSON.parse(storageItem);
  if (
    Array.isArray(savedValues.currentRatios) &&
    Array.isArray(savedValues.childStates) &&
    savedValues.currentRatios.length === childCount &&
    savedValues.childStates.length === childCount
  ) {
    return savedValues as { currentRatios: number[]; childStates: ResizePanelState[] };
  }

  return undefined;
}

/** Save a set of panel sizes with a key */
function saveState(key: string, currentRatios: number[], childStates: ResizePanelState[]) {
  localStorage.setItem(STORAGE_KEY_PREFIX + key, JSON.stringify({ currentRatios, childStates }));
}

/**
 * For an array of normalised panel sizes that are possibly undefined, spread the
 * undefined values over the remaining space
 */
function spreadUndefinedRatios(ratios: (number | undefined)[]): number[] {
  const unsetChildCount = ratios.reduce<number>((acc, ratio) => acc + (ratio === undefined ? 1 : 0), 0);
  const assignedSpace = ratios.reduce<number>((acc, ratio) => acc + (ratio ?? 0), 0);
  const unassignedSpace = 1 - assignedSpace;
  return ratios.map((ratio) => ratio ?? unassignedSpace / unsetChildCount);
}

export class ResizeContainerModel {
  /** The ratios of each panel before being minimized or maximized */
  private readonly lastSetRatios: number[];

  /** The minimum sizes of each child panel */
  private readonly minSizes: ReadonlyArray<number>;

  /** The maximum sizes of each child panel */
  private readonly maxSizes: ReadonlyArray<number>;

  /** The sum of the minimum sizes of all child panels */
  readonly minSize: number;

  /** The sum of the maximum sizes of all child panels */
  readonly maxSize: number;

  /** The state of each child panel */
  readonly childStates: ResizePanelState[];

  constructor(public childProps: ResizePanelProps[], private saveKey?: string) {
    // Attempt to load existing state
    const savedState = saveKey !== undefined ? loadState(saveKey, childProps.length) : undefined;

    // Use either loaded state or values from props
    if (savedState) {
      this.lastSetRatios = savedState.currentRatios;
      this.childStates = savedState.childStates;
    } else {
      this.lastSetRatios = spreadUndefinedRatios(childProps.map((props) => props.initialRatio));
      this.childStates = new Array(this.lastSetRatios.length).fill("default");
    }

    // Readonly values
    this.minSizes = childProps.map((props) => props.minSize ?? 0);
    this.minSize = this.minSizes.reduce((prev, curr) => prev + curr, 0);
    this.maxSizes = childProps.map((props) => props.maxSize ?? Infinity);
    this.maxSize = this.maxSizes.reduce((prev, curr) => prev + curr, 0);
  }

  /**
   * The current ratios of the child panels.
   * If a panel is set to `minimized` or `maximized` it's ratio will be set to
   * `0` or `Infinity` respectively.
   */
  get currentRatios(): number[] {
    return this.lastSetRatios.map((ratio, i) =>
      this.childStates[i] === "minimized" ? 0 : this.childStates[i] === "maximized" ? Infinity : ratio,
    );
  }

  /** Get the ratios of the panels with the min and max sizes applied */
  getDisplayRatios(containerSize: number): number[] {
    return fitChildren(containerSize, this.currentRatios, this.minSizes, this.maxSizes);
  }

  /** Get the new ratios of the children if the resize handle at the given index is moved to a new position */
  moveHandle(index: number, containerSize: number, position: number) {
    // Get normalised position of handle
    const targetHandlePos = this.currentRatios.slice(0, index + 1).reduce((curr, prev) => curr + prev, 0);

    // Calculate the amount to offset handle
    const normalisedOffset = position / containerSize - targetHandlePos;

    return moveHandle(index, normalisedOffset, containerSize, this.currentRatios, this.minSizes, this.maxSizes);
  }

  /** Set the current ratios of the panels */
  setRatios(ratios: number[]) {
    ratios.forEach((ratio, i) => (this.lastSetRatios[i] = ratio));
    this.saveState();
  }

  /** Set the state of a panel to make it minimized or maximized */
  setPanelState(index: number, state: ResizePanelState) {
    if (this.childStates[index] === state) {
      return;
    }

    this.childStates[index] = state;
    this.saveState();
  }

  /** Save the current state of the panels to local storage */
  private saveState() {
    if (this.saveKey) {
      saveState(this.saveKey, this.lastSetRatios, this.childStates);
    }
  }
}
