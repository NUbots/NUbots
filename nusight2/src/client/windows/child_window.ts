import { zip } from "@shared/array/zip";

import { bestFitColumns } from "@client/base/best_fit_columns";

/** A size value, either in pixels or as a percentage */
type Size = `${number}px` | `${number}%`;

export interface ChildWindowOpts {
  /** The absolute URL to open in the child window */
  url: string;
  /**
   * The name of the child window. If there's an existing window with the same name,
   * it will be reloaded with the new URL.
   */
  name: string;

  /** The width and height of the child window, overrides `aspectRatio` if set */
  size?: { width: Size; height: Size };

  /** The aspect ratio of the child window, used for best fit positioning when opening multiple windows */
  aspectRatio?: number;
}

function getPixelSize(windowSize: Size, containerPixels: number) {
  if (windowSize.endsWith("px")) {
    return Number(windowSize.replace("px", ""));
  }

  const percent = Number(windowSize.replace("%", ""));
  return containerPixels * (percent / 100);
}

/**
 * A wrapper class for managing child popup windows opened by a parent window.
 * Provides methods for opening and positioning one or more child windows,
 * focusing, closing, and listening for close events.
 */
export class ChildWindow {
  /** A reference to the browser window for the child */
  window: Window;

  /** Whether the window is closed */
  closed = false;

  /** The ID of the timeout used to check if the child window has closed */
  private childCloseTimeout: number;

  /** Listeners to run when the child window closes */
  private onCloseListeners = new Set<() => void>();

  /** Create a new `ChildWindow` wrapper for the given browser window */
  constructor(win: Window) {
    this.window = win;
    this.childCloseTimeout = window.setTimeout(this.checkForClose, 1000);
  }

  /** Open a single `ChildWindow` positioned with the given layout or at the center of the screen */
  static open(window: ChildWindowOpts, layout?: WindowLayout) {
    const win = openWindow(window.url, window.name, layout ?? centerWindow(window));
    return win ? new ChildWindow(win) : null;
  }

  /** Open multiple `ChildWindow`s positioned in a best-fit grid layout */
  static openInGrid(windows: Omit<ChildWindowOpts, "size">[]) {
    // When opening multiple child windows, we position them in a grid layout for best fit
    const windowLayouts = bestFitWindowsOnScreen({
      numItems: windows.length,
      itemAspectRatio: windows[0].aspectRatio ?? 1,
    });

    return zip(windows, windowLayouts).map(([winOpts, layout]) => {
      const win = openWindow(winOpts.url, winOpts.name, layout);
      return win ? new ChildWindow(win) : null;
    });
  }

  /** Focus the child window */
  focus() {
    this.window.focus();
  }

  /** Close the child window and dispose resources */
  close() {
    if (this.closed) {
      return;
    }

    this.window.close();
  }

  /** Add a listener to run when the child window closes */
  onClose(listener: () => void) {
    this.onCloseListeners.add(listener);
  }

  /** Check if the child window has closed, and run the closed handler */
  private checkForClose = () => {
    if (this.closed) {
      return;
    }

    if (this.window.closed) {
      this.handleWindowClosed();
    } else {
      this.childCloseTimeout = window.setTimeout(this.checkForClose, 1000);
    }
  };

  /** Handle the child window closing */
  private handleWindowClosed = () => {
    window.clearTimeout(this.childCloseTimeout);

    this.closed = true;

    for (const listener of this.onCloseListeners) {
      listener();
    }
  };
}

export interface WindowLayout {
  left: number;
  top: number;
  width: number;
  height: number;
}

/**
 * Calculate the best fit grid layout for the given number of items and aspect ratio.
 * Returns an array of window layouts (one for each item), ordered left to right, top to bottom.
 */
export function bestFitWindowsOnScreen(opts: { numItems: number; itemAspectRatio: number }): WindowLayout[] {
  const container = { width: screen.availWidth ?? screen.width, height: screen.availHeight ?? screen.height };

  const cols = bestFitColumns({
    container,
    itemAspectRatio: opts.itemAspectRatio,
    numItems: opts.numItems,
  });

  const rows = Math.ceil(opts.numItems / cols);

  const xGap = 24;
  const yGap = 24;
  const totalXGap = xGap * (cols - 1);
  const totalYGap = yGap * (rows - 1);
  const itemWidth = (container.width - totalXGap) / cols;
  const itemHeight = (container.height - totalYGap) / rows;

  return new Array(opts.numItems).fill(null).map((_, index) => {
    const col = index % cols;
    const row = Math.floor(index / cols);

    return {
      left: col * itemWidth + xGap * col,
      top: row * itemHeight + yGap * row,
      width: itemWidth,
      height: itemHeight,
    };
  });
}

/** Calculate a layout that will place a window with the given options at the center of the screen */
function centerWindow(opts: Pick<ChildWindowOpts, "size" | "aspectRatio">): WindowLayout {
  let width = 0;
  let height = 0;

  // If there is a size specified, use it
  if (opts.size) {
    width = getPixelSize(opts.size.width, screen.availWidth);
    height = getPixelSize(opts.size.height, screen.availHeight);
  }
  // Otherwise use a height of 80% of the screen height and a width calculated from the aspect ratio
  else {
    height = screen.availHeight * 0.8;
    width = height * (opts.aspectRatio ?? 1);
  }

  return {
    width,
    height,
    // Position the window roughly at the center of the screen
    left: (screen.availWidth - width) / 2,
    top: (screen.availHeight - height) / 2,
  };
}

/** Open and position a new popup window */
function openWindow(url: string, name: string, layout: WindowLayout) {
  // Open at 0, 0 so we can read the screenLeft and screenTop values.
  // This is necessary to properly position the window when there are multiple screens.
  // We will move the window to the correct position after it loads.
  const params = `top=0,left=0,width=${layout.width},height=${layout.height}`;

  // Add the `isChild` query param to the URL so the child knows it's a child window
  const urlObj = new URL(url, window.location.href);
  urlObj.searchParams.delete("isChild");
  urlObj.searchParams.append("isChild", "true");

  const childWindow = window.open(urlObj.toString(), name, params);

  // Position the window only on the first load
  // Note: this is only necessary because we might call `window.open()` with the window name of an existing window,
  // and don't want the existing window's position and size to be changed. In the future this check for `isLoaded` could
  // be removed by keeping track of existing windows and not calling `window.open()` if we already have a window
  // with the same name. Those windows could just be focused instead, with their URLs changed if necessary.
  if (childWindow && !childWindow.sessionStorage.getItem("isLoaded")) {
    childWindow.resizeTo(layout.width, layout.height);

    // Move the window to the correct position, taking screenLeft and screenTop into account.
    // Unfortunately, this doesn't work in Firefox, which always opens the child windows on the leftmost screen.
    childWindow.moveTo(childWindow.screenLeft + layout.left, childWindow.screenTop + layout.top);

    childWindow.addEventListener("load", () => {
      childWindow.sessionStorage.setItem("isLoaded", "true");
    });
  }

  return childWindow;
}
