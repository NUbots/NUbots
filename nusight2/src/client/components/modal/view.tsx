import React, { useEffect, useRef } from "react";

export type ModalProps = {
  children: React.ReactNode;
  closeWith?: {
    escapeKey?: boolean;
    closeButton?: boolean;
    backdropClick?: boolean;
  };
  onClose(): void;
};

const closeWithDefaults = {
  escapeKey: true,
  closeButton: true,
  backdropClick: false,
};

/** Shows a modal dialog (i.e. one that prevents interaction with content outside it until closed) */
export const Modal = (props: ModalProps) => {
  // Merge the defaults with the `closeWith` prop, allowing the prop to override defaults
  const closeWith = Object.assign({}, closeWithDefaults, props.closeWith);

  return (
    <div
      className="fixed flex items-center justify-center bg-[rgba(0,0,0,0.5)] h-full w-full top-0 left-0 z-10"
      tabIndex={-1}
      onClick={
        closeWith.backdropClick
          ? (event) => {
              // Close the modal when the backdrop is clicked, making sure that
              // the click is on the backdrop itself, not a child element
              if (event.target === event.currentTarget) {
                props.onClose();
              }
            }
          : undefined
      }
      onKeyDown={
        closeWith.escapeKey
          ? (event) => {
              // Close the modal when the escape key is pressed
              if (event.key === "Escape") {
                event.preventDefault();
                props.onClose();
              }
            }
          : undefined
      }
    >
      <FocusContainer>
        <div className="bg-gray-100 dark:bg-gray-900 rounded shadow-lg relative">
          {closeWith.closeButton ? (
            <button
              className="absolute right-0 top-0 p-2 rounded text-icon "
              data-id="modal-close-button"
              onClick={props.onClose}
            >
              <IconClose className="w-6 h-6" />
            </button>
          ) : null}
          {props.children}
        </div>
      </FocusContainer>
    </div>
  );
};

/**
 * An element that traps and circles focus within itself. Used for the modal dialog,
 * to prevent keyboard focus from leaving the modal when tabbing.
 */
function FocusContainer(props: { children: React.ReactNode }) {
  // Reference for the element that holds the focus container content
  const containerRef = useRef<HTMLDivElement>(null);

  // Reference for the element that was focused when the focus container was mounted
  const activeElementAtMount = useRef<Element | null>(null);

  useEffect(() => {
    // Keep track of the active element at mount
    activeElementAtMount.current = document.activeElement;

    // Move focus to the container
    if (containerRef.current) {
      const elementToFocus = containerRef.current.querySelector<HTMLElement>("[data-modal-autofocus]");

      // If there is an element explicitly marked for auto focus, focus it
      if (elementToFocus) {
        elementToFocus.focus();
      }
      // Otherwise focus the first focusable element
      else {
        redirectFocus("first", containerRef.current);
      }
    }

    // Restore focus to the original active element when the component is unmounted
    return () => {
      activeElementAtMount.current instanceof HTMLElement && activeElementAtMount.current.focus();
    };
  }, []);

  // Handle focus moving out of the container
  function onContainerBlur(event: React.FocusEvent) {
    if (!containerRef.current) {
      return;
    }

    // The element focus is moving to
    const newFocusedEl = event.relatedTarget as HTMLElement;

    // Focus moved to something not in the document: e.g. to another window
    if (!newFocusedEl) {
      return;
    }

    // Focus moved to the container itself or one of its children
    if (containerRef.current.contains(newFocusedEl)) {
      return;
    }

    // Focus moved to the before the container, redirect to the last focusable element in the container
    if (newFocusedEl.dataset.id === "focus-trap-before") {
      event.preventDefault();
      redirectFocus("last", containerRef.current);
    }
    // Focus moved to the after the container, redirect to the first focusable element in the container
    else if (newFocusedEl.dataset.id === "focus-trap-after") {
      event.preventDefault();
      redirectFocus("first", containerRef.current);
    }
  }

  return (
    <>
      <span tabIndex={0} data-id="focus-trap-before" className="sr-only" aria-hidden="true" />
      {/* Due to a React-specific implementation detail, onBlur() on the next element is actually listening
          for the 'focusout' event, which, unlike the actual 'blur' event, bubbles up from child elements -
          making it possible to listen for loss of focus anywhere inside the container, not just on the
          container itself */}
      <div
        ref={containerRef}
        tabIndex={0}
        onBlur={onContainerBlur}
        className="outline-none h-max flex items-center justify-center"
      >
        {props.children}
      </div>
      <span tabIndex={0} data-id="focus-trap-after" className="sr-only" aria-hidden="true" />
    </>
  );
}

/**
 * Redirect focus to the first or last keyboard-focusable element in the given parent,
 * or to the parent if there are no focusable children.
 */
function redirectFocus(direction: "first" | "last", parent: HTMLElement) {
  const { first, last } = getKeyboardFocusableElements(parent);

  if (direction === "first") {
    if (first) {
      first.focus();
    } else {
      parent.focus();
    }
  } else {
    if (last) {
      last.focus();
    } else {
      parent.focus();
    }
  }
}

/**
 * Get all the keyboard-focusable elements within the given element
 */
function getKeyboardFocusableElements(element: HTMLElement = document.body): {
  first?: HTMLElement;
  last?: HTMLElement;
} {
  const elements = Array.from(
    element.querySelectorAll<HTMLElement>(
      'a[href], button:not([data-id="modal-close-button"]), input, textarea, select, details, [tabindex]:not([tabindex="-1"])',
    ),
  ).filter((element) => !element.hasAttribute("disabled") && !element.getAttribute("aria-hidden"));

  return { first: elements[0], last: elements[elements.length - 1] };
}

function IconClose(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );
}
