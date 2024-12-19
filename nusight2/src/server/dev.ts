import compression from "compression";
import express from "express";
import http from "http";
import minimist from "minimist";
import favicon from "serve-favicon";
import { Server } from "socket.io";
import { createServer as createViteServer } from "vite";

import faviconPath from "../assets/favicon.ico";
import * as NUClearNetProxyParser from "../shared/nuclearnet/nuclearnet_proxy_parser";
import { VirtualRobots } from "../virtual_robots/virtual_robots";

import { NUsightServer } from "./nusight_server";
import { WebSocketServer } from "./web_socket/web_socket_server";

const args = minimist(process.argv.slice(2));

const withVirtualRobots = args["virtual-robots"] || false;
const nuclearnetAddress = args.address || "239.226.152.162";

async function main() {
  const app = express();
  const server = http.createServer(app);
  const sioNetwork = new Server(server, { parser: NUClearNetProxyParser });

  // Initialize socket.io namespace immediately to catch reconnections.
  NUsightServer.of(WebSocketServer.of(sioNetwork.of("/nuclearnet")), {
    fakeNetworking: withVirtualRobots,
    connectionOpts: { name: "nusight", address: nuclearnetAddress },
  });

  const viteDevServer = await createViteServer({
    server: {
      middlewareMode: true,
    },
  });

  app.use(compression());
  app.use(favicon(faviconPath));
  app.use(viteDevServer.middlewares);

  const port = process.env.PORT || 3000;

  server.listen(port, () => {
    // tslint:disable-next-line no-console
    console.log(`NUsight server started at http://localhost:${port}`);
  });

  if (withVirtualRobots) {
    const virtualRobots = VirtualRobots.of({
      fakeNetworking: true,
      nuclearnetAddress,
      numRobots: 3,
    });
    virtualRobots.start();
  }
}

main();
