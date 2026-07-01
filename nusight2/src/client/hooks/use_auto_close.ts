import { useEffect } from "react";

/** Close the current window when the parent window closes */
export function useAutoCloseOnParentClose(enabled: boolean) {
  useEffect(() => {
    if (!enabled) {
      return;
    }

    // There's no event for when the parent window closes,
    // so we setup a polling interval to check if it's closed
    const closeInterval = setInterval(() => {
      if (!window.opener || window.opener.closed) {
        clearInterval(closeInterval);
        window.close();
      }
    }, 1000);

    return () => clearInterval(closeInterval);
  }, [enabled]);
}
