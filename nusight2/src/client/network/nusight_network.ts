import { action } from 'mobx'
import { NUClearNetPacket } from 'nuclearnet.js'
import { NUClearNetOptions } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { NUClearNetSend } from 'nuclearnet.js'

import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { AppModel } from '../components/app/model'
import { RobotModel } from '../components/robot/model'
import { WebSocketProxyNUClearNetClient } from '../nuclearnet/web_socket_proxy_nuclearnet_client'

import { MessageTypePath } from './message_type_names'
import { RobotNetworkStatsModel } from './model'

/**
 * This class is intended to handle NUsight-specific networking. It handles the subscription of NUClearNet messages and
 * decoding them into real protobufjs objects for convenient use. Components should not directly use this class, but
 * instead create their own ComponentNetwork class which uses the Network helper class.
 */
export class NUsightNetwork {
  constructor(
    private nuclearnetClient: NUClearNetClient,
    private appModel: AppModel,
    private messageTypePath: MessageTypePath,
  ) {}

  static of(appModel: AppModel) {
    const messageTypePath = MessageTypePath.of()
    const nuclearnetClient: NUClearNetClient = WebSocketProxyNUClearNetClient.of()
    return new NUsightNetwork(nuclearnetClient, appModel, messageTypePath)
  }

  connect(opts: NUClearNetOptions): () => void {
    return this.nuclearnetClient.connect(opts)
  }

  send(opts: NUClearNetSend) {
    this.nuclearnetClient.send(opts)
  }

  onNUClearMessage<T>(messageType: MessageType<T>, cb: MessageCallback<T>) {
    const messageTypeName = this.messageTypePath.getPath(messageType)
    return this.nuclearnetClient.on(
      messageTypeName,
      action((packet: NUClearNetPacket) => {
        const buffer = new Uint8Array(packet.payload)
        const message = messageType.decode(buffer)
        const peer = packet.peer
        const robotModel = this.appModel.robots.find(robot => {
          return (
            robot.name === peer.name && robot.address === peer.address && robot.port === peer.port
          )
        })
        if (robotModel) {
          const stats = RobotNetworkStatsModel.of(robotModel)
          stats.packets += 1
          stats.packetsPerSecond.update(1)
          stats.bytes += buffer.length
          stats.bytesPerSecond.update(buffer.length)

          cb(robotModel, message)
        }
      }),
    )
  }

  onNUClearJoin(cb: (peer: NUClearNetPeer) => void) {
    this.nuclearnetClient.onJoin(cb)
  }

  onNUClearLeave(cb: (peer: NUClearNetPeer) => void) {
    this.nuclearnetClient.onLeave(cb)
  }
}

export interface MessageType<T> {
  new (...args: any[]): T

  decode(...args: any[]): T
}

export type MessageCallback<T> = (robotModel: RobotModel, message: T) => void
