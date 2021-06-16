// See https://github.com/socketio/socket.io-protocol
export enum TYPES {
  CONNECT = 0,
  DISCONNECT = 1,
  EVENT = 2,
  ACK = 3,
  ERROR = 4,
  BINARY_EVENT = 5,
  BINARY_ACK = 6,
}

export interface ConnectPacket {
  nsp: string
  type: TYPES.CONNECT
}

export interface DisconnectPacket {
  nsp: string
  type: TYPES.DISCONNECT
}

export interface EventPacket {
  nsp: string
  type: TYPES.EVENT
  data: any[]
  id: number
}

export interface AckPacket {
  nsp: string
  type: TYPES.ACK
  data: any[]
  id: number
}

export interface ErrorPacket {
  nsp: string
  type: TYPES.ERROR
  data: any
}

export interface BinaryEventPacket {
  nsp: string
  type: TYPES.BINARY_EVENT
  data: any[]
  id: number
}

export interface BinaryAckPacket {
  nsp: string
  type: TYPES.BINARY_ACK
  data: any[]
  id: number
}

export type Packet =
  | ConnectPacket
  | DisconnectPacket
  | EventPacket
  | AckPacket
  | ErrorPacket
  | BinaryEventPacket
  | BinaryAckPacket
