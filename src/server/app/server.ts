import Server = SocketIO.Server
import { inject } from 'inversify'
import { NUClearNet } from 'nuclearnet.js'
import { NUClearNetPeer } from 'nuclearnet.js'
import { Client } from './client'
import { Robot } from './robot'
import Socket = SocketIO.Socket

export class NUSightServer {
  private clients: Client[]
  private robots: Robot[]

  public constructor(@inject(NUClearNet) private nuclearNetwork: NUClearNet,
                     private sioNetwork: Server) {
    this.clients = []
    this.robots = []

    this.nuclearNetwork.on('nuclear_join', this.onNUClearJoin)
    this.nuclearNetwork.on('nuclear_leave', this.onNUClearLeave)
    this.sioNetwork.on('connection', this.onClientConnection)
  }

  public connect() {
    this.nuclearNetwork.connect({ name: 'nusight' })
  }

  private onNUClearJoin = (peer: NUClearNetPeer) => {
    // tslint:disable-next-line no-console
    console.log(`Peer "${peer.name}" from ${peer.address} joined the NUClear network.`)
    const robot = Robot.of({
      name: peer.name,
    })
    this.robots.push(robot)
  }

  private onNUClearLeave = (peer: NUClearNetPeer) => {
    // tslint:disable-next-line no-console
    console.log(`Peer "${peer.name}" from ${peer.address} left the NUClear network.`)
    const index = this.robots.findIndex(robot => robot.name === peer.name)
    if (index >= 0) {
      this.robots.splice(index, 1)
    }
  }

  private onClientConnection = (sioSocket: Socket) => {
    const address = sioSocket.client.conn.remoteAddress
    // tslint:disable-next-line no-console
    console.log(`Client "${sioSocket.id}" from ${address} connected (Total: ${this.clients.length + 1})`)
    const client = Client.of({
      nuclearNetwork: this.nuclearNetwork,
      sioSocket,
      onDisconnectCallback: () => {
        this.clients.splice(this.clients.indexOf(client), 1)
        // tslint:disable-next-line no-console
        console.log(`Client "${sioSocket.id}" from ${address} disconnected (Total: ${this.clients.length})`)
      },
    })
    this.clients.push(client)
  }
}
