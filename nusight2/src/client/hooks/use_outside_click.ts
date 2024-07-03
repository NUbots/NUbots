import { useEffect } from "react";

/**
 * Calls the given callback when a click occurs on an element that is outside the given ref's element
 * (e.g. not the element itself or one of its descendants).
 */
export function useOutsideClick(ref: React.RefObject<HTMLElement>, callback: (event: MouseEvent) => void) {
  useEffect(() => {
    function handleDocumentClick(event: MouseEvent) {
      if (!ref.current || ref.current.contains(event.target as Node)) {
        return;
      }

      callback(event);
    }

    document.addEventListener("click", handleDocumentClick);

    return () => {
      document.removeEventListener("click", handleDocumentClick);
    };
  }, [ref]);
}
