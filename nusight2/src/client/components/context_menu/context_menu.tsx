import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";

import { useOutsideClick } from "../../hooks/use_outside_click";
import { FocusContainer } from "../focus_container/focus_container";
import { Icon } from "../icon/view";

export interface ContextMenuDivider {
  /** Horizontal bar in a context menu */
  type: "divider";
}

export interface ContextMenuSubMenu {
  type: "submenu";
  title: string;
  icon?: React.ReactNode | string;
  items: ContextMenuItem[];
}

export interface ContextMenuOption {
  /**
   * A button in a context menu that performs an action when clicked. This
   * is the default menu item type.
   */
  type?: "button";
  /** Name for the menu option */
  title: string;
  /** Hint text for the menu option */
  hint?: string;
  /** Icon for the menu option. Either the name of a material icon or a React element */
  icon?: React.ReactNode | string;
  /** Whether this option is disabled in the menu */
  disabled?: boolean;
  /** The action to perform when this option is selected */
  action: () => void;
}

export type ContextMenuItem = ContextMenuDivider | ContextMenuSubMenu | ContextMenuOption;

interface PositionalEvent {
  pageY: number;
  pageX: number;
  preventDefault?: () => void;
}

interface ContextMenuProps {
  positionX: number;
  positionY: number;
  items: ContextMenuItem[];
  onClose: () => void;
}

interface ContextMenuContext {
  setProps: (props?: ContextMenuProps) => void;
}

const contextMenuContext = createContext<ContextMenuContext | null>(null);

// A delay so the submenu doesn't disappear immediately on mouse out, allowing it to
// be reached over empty space. The same length delay is used for the options appearing,
// preventing multiple submenus from being visible at once. This also allows skipping the
// the sub menu when quickly through the parent menu options.
const subMenuDelayMillis = 250;

/**
 * Returns a callback that opens a context menu from a mouse event. This is typically used by binding to the
 * `onContextMenu` property. The callback is memoized to the given dependencies.
 *
 * Any arguments required by the `getItems` callback must be passed into the returned callback.
 *
 * The user of this hook must be a descendent of the <ContextMenuProvider> component.
 */
export function useContextMenu<T extends [...any[]]>(
  getItems: (...args: T) => ContextMenuItem[],
  deps: React.DependencyList,
) {
  const context = useContext(contextMenuContext);
  return useCallback(
    (event: PositionalEvent, ...args: T) => {
      event.preventDefault?.();

      if (!context) {
        console.warn("Unable to show context menu: ContextMenuContext not found.");
        return;
      }

      context.setProps({
        positionX: event.pageX,
        positionY: event.pageY,
        items: getItems(...args),
        onClose: () => context.setProps?.(undefined),
      });
    },
    [...deps, context?.setProps],
  );
}

/**
 * Provides the React context for context menus, and an outlet to render the currently open menu.
 * This component must be an ancestor of the element attempting to show a context menu.
 */
export function ContextMenuProvider(props: { children: React.ReactNode }) {
  const [contextMenuProps, setContextMenuProps] = useState<ContextMenuProps | undefined>();
  const value = useMemo(() => ({ setProps: setContextMenuProps }), [setContextMenuProps]);
  return (
    <contextMenuContext.Provider value={value}>
      {props.children}
      {contextMenuProps ? <ContextMenu {...contextMenuProps} /> : null}
    </contextMenuContext.Provider>
  );
}

/** A menu with a fixed position on the page */
function ContextMenu(props: ContextMenuProps) {
  const menuRef = useRef<HTMLDivElement>(null);
  const [menuPosition, setMenuPosition] = useState<{ top: number; left: number }>();

  // Close when window focus is lost
  useEffect(() => {
    window.addEventListener("blur", props.onClose);
    return () => window.removeEventListener("blur", props.onClose);
  }, [props.onClose]);

  // Close when clicking outside the menu
  useOutsideClick(menuRef, props.onClose);

  // Before rendering, check the size of the menu and clamp it inside the window area
  useLayoutEffect(() => {
    const menu = menuRef.current;
    if (!menu) {
      return;
    }

    const yIsOverflowing = props.positionY + menu.clientHeight > window.innerHeight;

    setMenuPosition({
      left: Math.min(props.positionX + 2, window.innerWidth - menu.clientWidth),
      top: props.positionY + (yIsOverflowing ? -(menu.clientHeight + 2) : 2),
    });
  }, [props.positionX, props.positionY]);

  return (
    // Outer container to block mouse events outside of the menu while the menu is open
    <div
      onContextMenu={(e) => {
        e.preventDefault();
      }}
      className="fixed left-0 top-0 w-screen h-screen z-50"
    >
      <FocusContainer>
        <div
          data-focus-container-autofocus
          ref={menuRef}
          tabIndex={-1}
          style={menuPosition}
          className="absolute flex flex-col min-w-[18rem] shadow-auto-card text-sm text-auto-primary bg-auto-surface-2 rounded py-1 focus:outline-none select-none"
          onKeyDown={(e) => e.key === "Escape" && props.onClose()}
        >
          {props.items.map((option, i) =>
            option.type === "divider" ? (
              <div key={i} className="border-b border-auto my-1" />
            ) : option.type === "submenu" ? (
              <ContextMenuSubMenuOption key={option.title} option={option} onActionDone={props.onClose} />
            ) : (
              <ContextMenuItem key={option.title} option={option} onActionDone={props.onClose} />
            ),
          )}
        </div>
      </FocusContainer>
    </div>
  );
}

function ContextMenuSubMenuOption({ option, onActionDone }: { option: ContextMenuSubMenu; onActionDone: () => void }) {
  const [showChildren, setShowChildren] = useState(false);
  const timeoutHandle = useRef<number>();

  const showChildMenu = useCallback(() => {
    clearTimeout(timeoutHandle.current);
    timeoutHandle.current = window.setTimeout(() => setShowChildren(true), subMenuDelayMillis);
  }, [setShowChildren]);

  const hideChildMenu = useCallback(() => {
    clearTimeout(timeoutHandle.current);
    timeoutHandle.current = window.setTimeout(() => setShowChildren(false), subMenuDelayMillis);
  }, [setShowChildren]);

  return (
    <div
      className="group relative flex h-8 gap-2 px-2 text-left items-center focus:bg-auto-contrast-2 hover:bg-auto-contrast-2 disabled:text-auto-disabled whitespace-nowrap"
      onMouseEnter={showChildMenu}
      onMouseLeave={hideChildMenu}
      onFocus={showChildMenu}
      onBlur={hideChildMenu}
      tabIndex={0}
    >
      <div className="inline-flex w-6 items-center shrink-0">
        {typeof option.icon === "string" ? (
          <Icon size="20" className="text-auto-icon mt-px">
            {option.icon}
          </Icon>
        ) : (
          option.icon
        )}
      </div>
      <span className="flex-grow">{option.title}</span>
      <Icon size={20}>chevron_right</Icon>
      {showChildren ? <ContextMenuSubMenuItems items={option.items} onActionDone={onActionDone} /> : null}
    </div>
  );
}

function ContextMenuSubMenuItems({ items, onActionDone }: { items: ContextMenuItem[]; onActionDone: () => void }) {
  const menuRef = useRef<HTMLDivElement>(null);
  const [menuPosition, setMenuPosition] = useState<{ top?: number; bottom?: number; left?: string; right?: string }>({
    top: -4,
    left: "100%",
  });

  // Before rendering, check the size of the menu and clamp it inside the window area
  useLayoutEffect(() => {
    const menu = menuRef.current;
    if (!menu) {
      return;
    }

    const area = menu.getBoundingClientRect();

    const yIsOverflowing = area.y + menu.clientHeight > window.innerHeight;
    const xIsOverflowing = area.x + menu.clientWidth > window.innerWidth;

    setMenuPosition({
      top: yIsOverflowing ? undefined : -4,
      bottom: yIsOverflowing ? -4 : undefined,
      left: xIsOverflowing ? undefined : "100%",
      right: xIsOverflowing ? "100%" : undefined,
    });
  }, []);

  return (
    <div
      ref={menuRef}
      style={menuPosition}
      className="absolute flex flex-col min-w-[18rem] shadow-auto-card bg-auto-surface-2 rounded py-1"
    >
      {items.map((option, i) =>
        option.type === "divider" ? (
          <div key={i} className="border-b border-auto my-1" />
        ) : option.type === "submenu" ? (
          <ContextMenuSubMenuOption key={option.title} option={option} onActionDone={onActionDone} />
        ) : (
          <ContextMenuItem key={option.title} option={option} onActionDone={onActionDone} />
        ),
      )}
    </div>
  );
}

function ContextMenuItem({ option, onActionDone }: { option: ContextMenuOption; onActionDone: () => void }) {
  return (
    <button
      disabled={option.disabled}
      onClick={() => {
        option.action();
        onActionDone();
      }}
      className="flex h-8 gap-2 pl-2 pr-4 items-center text-left focus:bg-auto-contrast-2 enabled:hover:bg-auto-contrast-2 disabled:text-auto-disabled whitespace-nowrap"
    >
      <div className="inline-flex w-6 items-center justify-center shrink-0">
        {typeof option.icon === "string" ? (
          <Icon size="20" className="text-auto-icon mt-px">
            {option.icon}
          </Icon>
        ) : (
          option.icon
        )}
      </div>
      <span className="flex-grow">{option.title}</span>
      <span className="text-auto-hint">{option.hint}</span>
    </button>
  );
}
