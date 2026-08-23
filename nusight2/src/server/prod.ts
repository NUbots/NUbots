import compression from "compression";
import history from "connect-history-api-fallback";
import express from "express";
import http from "http";
import minimist from "minimist";
import * as path from "path";
import favicon from "serve-favicon";
import { Server } from "socket.io";

import * as NUClearNetProxyParser from "../shared/nuclearnet/nuclearnet_proxy_parser";
import { VirtualRobots } from "../virtual_robots/virtual_robots";

import { NUsightServer } from "./nusight_server";
import { parseNonNegativeIntArg } from "./parse_non_negative_int_arg";
import { WebSocketServer } from "./web_socket/web_socket_server";

const args = minimist(process.argv.slice(2));
const withVirtualRobots = args["virtual-robots"] || false;
const nuclearnetAddress = args.address || "239.226.152.162";
const nuclearnetPort = args.port || "7447";

// Optional RoboCup UDP side channel: presents robots (or other teams) sending serialised RoboCup
// team communication packets to this port as additional NUsight peers. This is typically pointed
// at the same port configured for team communication in the RobotCommunication module, which
// defaults to 10000 + team_id. We're team 1, so default to that port here too; override with
// --robocup-team-id if playing under a different team number, or --robocup-udp-port directly.
// Pass --robocup-udp-port=0 to disable the side channel entirely.
const robocupTeamId = parseNonNegativeIntArg(args["robocup-team-id"], "--robocup-team-id") ?? 1;
const robocupUDPPort =
  args["robocup-udp-port"] !== undefined
    ? parseNonNegativeIntArg(args["robocup-udp-port"], "--robocup-udp-port")
    : 10000 + robocupTeamId;
const robocupUDPAllowedAddresses: string[] | undefined = args["robocup-udp-allowed-addresses"]
  ? String(args["robocup-udp-allowed-addresses"])
      .split(",")
      .map((address) => address.trim())
      .filter(Boolean)
  : undefined;
const robocupUDP = robocupUDPPort
  ? { port: robocupUDPPort, allowedAddresses: robocupUDPAllowedAddresses }
  : undefined;

const app = express();
const server = http.createServer(app);
const sioNetwork = new Server(server, { parser: NUClearNetProxyParser } as any);

app.use(
  history({
    rewrites: [
      // Allows user to navigate to /storybook/ without needing to type /index.html
      { from: /\/storybook\/$/, to: "storybook/index.html" },
      // Additional entry points that are not the main `index.html`.
      // Entries here should also be configured in `vite.config.ts` and in `dev.ts`.
      { from: /\/standalone(?!.*\..*)/, to: "standalone.html" },
    ],
  }),
);
app.use(compression());
app.use(express.static(path.join("dist")));
app.use(favicon(path.join("dist", "favicon.ico")));

const port = process.env.PORT || 9090;
server.listen(port, () => {
  // tslint:disable-next-line no-console
  console.log(`NUsight server started at http://localhost:${port}`);
});

if (withVirtualRobots) {
  const virtualRobots = VirtualRobots.of({ fakeNetworking: true, nuclearnetAddress, numRobots: 3 });
  virtualRobots.start();
}

NUsightServer.of(WebSocketServer.of(sioNetwork.of("/nuclearnet")), {
  fakeNetworking: withVirtualRobots,
  connectionOpts: { name: "nusight", address: nuclearnetAddress, port: nuclearnetPort },
  robocupUDP,
});
