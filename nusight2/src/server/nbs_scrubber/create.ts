import { NUsightSession } from "../session/session";

import { NbsScrubberNetwork } from "./network";

export function createNbsScrubber(session: NUsightSession) {
  const scrubberNetwork = new NbsScrubberNetwork(session);
  return () => scrubberNetwork.destroy();
}
