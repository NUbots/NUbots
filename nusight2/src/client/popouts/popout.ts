import { createContext, useCallback, useContext, useEffect, useMemo, useState } from "react";
import { bestFitWindowsOnScreen, ChildWindow, ChildWindowOpts, WindowLayout } from "@client/windows/child_window";
import { CrossWindowMessenger } from "@client/windows/cross_window_messenger";
import { zip } from "@shared/array/zip";
import { action, computed, IObservableArray, observable } from "mobx";

import { popoutBroadcastChannel, PopoutChildRendered, PopoutParentRendered } from "./messaging";

export interface UsePopoutParentOpts {
  /**
   * The name of this popout, used to connect the parent to its child.
   * Should be unique for a specific parent-child, e.g.`vision-camera/CentreWide`.
   */
  name: string;

  /** Called when the popout child has rendered and established a connection */
  onChildRendered: (message: PopoutChildRendered, source: Window) => void;
}

export function usePopoutParent(opts: UsePopoutParentOpts) {
  const { name, onChildRendered } = opts;

  // Create a messenger to communicate with child windows
  const messenger = usePopoutMessenger();

  // Listen for messages from child windows
  useEffect(() => {
    return messenger.on(PopoutChildRendered, (message, source) => {
      if (!source) {
        console.warn("Received a PopoutChildRendered message without a source window", { message, source });
        return;
      }

      if (message.name === name) {
        onChildRendered(message, source as Window);
      }
    });
  }, [messenger, name, onChildRendered]);

  // On render, notify potential child windows to establish a connection
  useEffect(() => {
    messenger.send(new PopoutParentRendered({ name }));
  }, [messenger, name]);
}

interface UsePopoutChildOpts {
  /** Called when the popout parent has rendered and established a connection */
  onParentRendered?: (message: PopoutParentRendered) => void;
}

/** Setup a popout child to communicate with its parent */
export function usePopoutChild(opts: UsePopoutChildOpts = {}) {
  const { onParentRendered } = opts;

  // Create a messenger to communicate with parent windows
  const messenger = usePopoutMessenger();

  // Listen for messages from parent windows
  useEffect(() => {
    // `window.name` should have been set by the parent window when it opened this window
    if (!window.name) {
      return;
    }

    return messenger.on(PopoutParentRendered, (message) => {
      if (message.name === window.name) {
        // Notify the parent window that we are rendered
        messenger.sendToParent(new PopoutChildRendered({ name: window.name }));

        // Notify our parent component that our parent window has rendered
        onParentRendered?.(message);
      }
    });
  }, [messenger, window.name, onParentRendered]);

  // Notify potential parent windows that a child has rendered
  useEffect(() => {
    // `window.name` should have been set by the parent window when it opened this window
    if (!window.name) {
      return;
    }

    messenger.sendToParent(new PopoutChildRendered({ name: window.name }));
  }, [messenger, window.name]);
}

export function usePopoutMessenger() {
  const messenger = useMemo(() => new CrossWindowMessenger(popoutBroadcastChannel), []);

  // Destroy the messenger when it's recreated or this component is unmounted
  useEffect(() => () => messenger.destroy(), [messenger]);

  return messenger;
}

export const PopoutGroupContext = createContext<PopoutGroup | undefined>(undefined);

/** Holds a child window and provides methods to open, focus, and close it */
export class Popout {
  /** The options used to create the child window */
  @observable.ref windowOpts: ChildWindowOpts;

  /** A reference to the child window, if open */
  @observable.ref childWindow: ChildWindow | null = null;

  constructor(windowOpts: ChildWindowOpts) {
    this.windowOpts = windowOpts;
  }

  /** Whether the popout child window is open */
  @computed
  get isOpen() {
    return this.childWindow !== null;
  }

  /** Attach the given child window to this popout */
  @action.bound
  attachChildWindow(child: ChildWindow) {
    child.onClose(
      action(() => {
        this.childWindow = null;
      }),
    );
    this.childWindow = child;
  }

  /** Open the popout child window, or focus it if already open */
  @action.bound
  open(layout?: WindowLayout) {
    if (this.childWindow) {
      this.childWindow.focus();
      return;
    }

    const child = ChildWindow.open(this.windowOpts, layout);
    if (!child) {
      console.warn("Failed to open popout window", this.windowOpts.name);
      return;
    }

    this.attachChildWindow(child);
  }

  /** Focus the popout child window if open */
  @action.bound
  focus() {
    this.childWindow?.focus();
  }

  /** Close the popout child window if open */
  @action.bound
  close() {
    this.childWindow?.close();
  }
}

/** Create a popout instance to manage a child window */
export function usePopout(
  windowOpts: ChildWindowOpts,
  opts?: {
    onChildRendered?: (message: PopoutChildRendered, source: Window) => void;
  },
): Popout {
  const [popout] = useState<Popout>(() => new Popout(windowOpts));

  // Update the popout window options if they change
  useEffect(() => {
    popout.windowOpts = windowOpts;
  }, [popout, windowOpts?.name, windowOpts.url, windowOpts.size, windowOpts.aspectRatio]);

  const onChildRendered = useCallback(
    (message: PopoutChildRendered, source: Window) => {
      if (message.name === windowOpts.name) {
        // Attach the child window if it's not already attached to the popout
        if (popout.childWindow?.window !== source) {
          popout.attachChildWindow(new ChildWindow(source));
        }

        // Notify the parent component that the child has rendered
        opts?.onChildRendered?.(message, source);
      }
    },
    [popout, windowOpts.name, opts?.onChildRendered],
  );

  // Add this popout to the closest group in context if there's one
  const group = useContext(PopoutGroupContext);
  useEffect(() => group?.addPopout(popout), [group, popout]);

  // Setup communication between the parent and child windows
  usePopoutParent({ name: windowOpts.name, onChildRendered });

  return popout;
}

/** Holds a collection of popouts and allows for opening and closing them together */
export class PopoutGroup {
  /**
   * The popouts in this group, mapped into individual lists of popouts that share a name.
   * Each popout name can have multiple popout instances. For example, if the same camera is
   * rendered as a thumbnail and a full view, there'll be two popouts with the same name.
   */
  @observable
  private popoutsByName: Map<string, IObservableArray<Popout>> = new Map();

  /** The first popout of each name in this group, used to open and close all popouts */
  @computed
  private get popoutsFirstPerName(): Popout[] {
    return Array.from(this.popoutsByName.values()).map((popouts) => popouts[0]);
  }

  /** The number of uniquely named popouts in this group */
  @computed
  get count() {
    return this.popoutsByName.size;
  }

  /** The number of uniquely named popouts that are open in this group */
  @computed
  get countOpen() {
    let count = 0;

    for (const group of this.popoutsByName.values()) {
      if (group.some((popout) => popout.isOpen)) {
        count++;
      }
    }

    return count;
  }

  /** Add a popout to this group. Returns a function to remove the popout. */
  @action.bound
  addPopout(child: Popout) {
    const popouts = this.popoutsByName.get(child.windowOpts.name) ?? observable.array<Popout>([]);
    popouts.push(child);

    this.popoutsByName.set(child.windowOpts.name, popouts);

    return action(() => {
      popouts.remove(child);

      if (popouts.length === 0) {
        this.popoutsByName.delete(child.windowOpts.name);
      }
    });
  }

  /** Open all popouts in this group */
  @action.bound
  openAll() {
    if (this.popoutsByName.size === 0) {
      return;
    }

    // Open the first popout of each name. Other popouts of the same name
    // will detect when the first one opens and set themselves as open.
    const popouts = this.popoutsFirstPerName;

    const windowLayouts = bestFitWindowsOnScreen({
      numItems: popouts.length,
      itemAspectRatio: popouts[0].windowOpts.aspectRatio ?? 1,
    });

    for (const [popout, layout] of zip(popouts, windowLayouts)) {
      popout.open(layout);
    }
  }

  /** Close all popouts in this group */
  @action.bound
  closeAll() {
    // Close the first popout of each name. Other popouts of the same name
    // will detect when the first one closes and set themselves as closed.
    for (const popout of this.popoutsFirstPerName) {
      popout.close();
    }
  }
}
