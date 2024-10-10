import { NUsightSession } from "../session/session";

import { FilePickerNetwork } from "./network";

export function createFilePicker(session: NUsightSession) {
  const filePickerNetwork = new FilePickerNetwork(session);
  return () => filePickerNetwork.destroy();
}
