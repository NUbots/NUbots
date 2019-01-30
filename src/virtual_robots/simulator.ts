import { NUClearNetClient } from '../shared/nuclearnet/nuclearnet_client'

export interface PeriodicSimulator {
  simulate(time: number, index: number, numRobots: number): Message[]
}

export interface Simulator {
  start(network: NUClearNetClient): () => void
}

export interface Message {
  messageType: string
  buffer: Uint8Array
  reliable?: boolean
}
