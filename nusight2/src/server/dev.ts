import fs from "node:fs";
import path from "node:path";

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
const nuclearnetPort = args.port || "7447";

// Optional Overview UDP side channel: presents robots sending serialised Overview packets to this
// port as additional NUsight peers. Disabled unless a port is provided.
const overviewUDPPort = args["overview-udp-port"] ? Number(args["overview-udp-port"]) : undefined;
const overviewUDPAllowedAddresses: string[] | undefined = args["overview-udp-allowed-addresses"]
  ? String(args["overview-udp-allowed-addresses"])
      .split(",")
      .map((address) => address.trim())
      .filter(Boolean)
  : undefined;
const overviewUDP = overviewUDPPort
  ? { port: overviewUDPPort, allowedAddresses: overviewUDPAllowedAddresses }
  : undefined;

// Optional RoboCup UDP side channel: presents robots (or other teams) sending serialised RoboCup
// team communication packets to this port as additional NUsight peers. This is typically pointed
// at the same port configured for team communication in the RobotCommunication module (defaults
// to 10000 + team_id). Disabled unless a port is provided.
const robocupUDPPort = args["robocup-udp-port"] ? Number(args["robocup-udp-port"]) : undefined;
const robocupUDPAllowedAddresses: string[] | undefined = args["robocup-udp-allowed-addresses"]
  ? String(args["robocup-udp-allowed-addresses"])
      .split(",")
      .map((address) => address.trim())
      .filter(Boolean)
  : undefined;
const robocupUDP = robocupUDPPort
  ? { port: robocupUDPPort, allowedAddresses: robocupUDPAllowedAddresses }
  : undefined;

async function main() {
  const app = express();
  const server = http.createServer(app);
  const sioNetwork = new Server(server, { parser: NUClearNetProxyParser });

  // Initialize socket.io namespace immediately to catch reconnections.
  NUsightServer.of(WebSocketServer.of(sioNetwork.of("/nuclearnet")), {
    fakeNetworking: withVirtualRobots,
    connectionOpts: { name: "nusight", address: nuclearnetAddress, port: nuclearnetPort },
    overviewUDP,
    robocupUDP,
  });

  const viteDevServer = await createViteServer({
    server: {
      middlewareMode: true,
    },
  });

  app.use(compression());
  app.use(favicon(faviconPath));

  // Configure extra client entry points in addition to the main `index.html`.
  // Entries specified here should also be configured in `vite.config.ts` and in `prod.ts`.
  const additionalEntryPoints = {
    "/standalone": path.resolve("standalone.html"),
  } as const;

  for (const [url, htmlFilePath] of Object.entries(additionalEntryPoints)) {
    app.use(url, async (_req, res, next) => {
      try {
        const template = await fs.promises.readFile(htmlFilePath, "utf-8");
        const html = await viteDevServer.transformIndexHtml(url, template);
        res.status(200).set({ "Content-Type": "text/html" }).end(html);
      } catch (e) {
        next(e);
      }
    });
  }

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
