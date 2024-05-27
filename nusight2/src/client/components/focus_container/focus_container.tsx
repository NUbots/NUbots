import React, { useEffect, useRef } from "react";

/**
 * An element that traps and circles focus within itself. Used to prevent keyboard focus
 * from leaving the container when tabbing.
 */
export function FocusContainer(props: { children: React.ReactNode }) {
  // Reference for the element that holds the focus container content
  const containerRef = useRef<HTMLDivElement>(null);

  // Reference for the element that was focused when the focus container was mounted
  const activeElementAtMount = useRef<Element | null>(null);

  useEffect(() => {
    // Keep track of the active element at mount
    activeElementAtMount.current = document.activeElement;

    // Move focus to the container
    if (containerRef.current) {
      const elementToFocus = containerRef.current.querySelector<HTMLElement>("[data-focus-container-autofocus]");

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
      // If the focus is no longer in the container, we assume it was programmatically moved out,
      // and therefore skip restoring focus here
      if (!containerRef.current?.contains(document.activeElement)) {
        return;
      }

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

    // Focus moved to the trap before the container, redirect to the last focusable element in the container
    if (newFocusedEl.dataset.id === "focus-trap-before") {
      event.preventDefault();
      redirectFocus("last", containerRef.current);
    }
    // Focus moved to the trap after the container, redirect to the first focusable element in the container
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
        tabIndex={-1}
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
      'a[href], button:not([data-focus-container-close-button]), input, textarea, select, details, [tabindex]:not([tabindex="-1"])',
    ),
  ).filter((element) => !element.hasAttribute("disabled") && !element.getAttribute("aria-hidden"));

  return { first: elements[0], last: elements[elements.length - 1] };
}
