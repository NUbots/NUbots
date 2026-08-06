import { MessageInstance } from "./types";

export interface EmitOptions {
  target?: string;
  reliable?: boolean;
}

declare function emit(message: MessageInstance, options?: EmitOptions): void;

export type Emit = typeof emit;
export type EmitAsync = (...args: Parameters<Emit>) => Promise<void>;
