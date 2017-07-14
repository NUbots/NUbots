export interface Simulator {
  simulate(time: number, index: number, numRobots: number): Message[]
}

export interface Message {
  messageType: string
  buffer: Uint8Array
  reliable?: boolean
}
