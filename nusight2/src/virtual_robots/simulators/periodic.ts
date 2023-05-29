import { now } from "mobx-utils";

export function periodic(frequency: number) {
  return now(1000 / frequency) / 1000;
}
