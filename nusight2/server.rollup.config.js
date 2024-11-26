/* eslint-env node */
// @ts-check
import { nodeResolve } from "@rollup/plugin-node-resolve";
import run from "@rollup/plugin-run";
import typescript from "@rollup/plugin-typescript";
import url from "@rollup/plugin-url";
import minimist from "minimist";
import * as path from "path";

const rootDir = import.meta.dirname;

const isProduction = process.env.BUILD === "production";
const isContinuousIntegration = Boolean(process.env.CI);

function getArgs() {
  const args = minimist(process.argv.slice(2), {
    boolean: ["run", "virtual-robots", "inspect"],
    string: ["address"],
  });

  const nonRollupArgs = [];

  if (args["virtual-robots"]) {
    nonRollupArgs.push("--virtual-robots");
  }

  if (args["address"]) {
    nonRollupArgs.push("--address", args["address"]);
  }

  return {
    run: Boolean(args["run"]),
    inspect: Boolean(args["inspect"]),
    nonRollupArgs,
  };
}

const args = getArgs();

let assetsDir = path.join(rootDir, "dist", "assets");

if (process.platform === "win32") {
  assetsDir = toUnixPath(assetsDir);
}

const config = {
  input: path.join(rootDir, "src", "server", isProduction ? "prod.ts" : "dev.ts"),
  output: {
    dir: path.join(rootDir, "dist"),
    format: "es",
    name: "NUsightServer",
    sourcemap: isContinuousIntegration ? false : true,
  },
  plugins: [
    url({
      include: ["**/*.ico", "**/*.jpg"],
      publicPath: assetsDir + "/", // Prepended to the path of imported assets
      destDir: assetsDir, // Put copied assets here
      limit: 0, // Don't inline any assets, always copy and link
    }),
    typescript({
      include: ["./src/**/*"],
      exclude: ["./node_modules/**/*", "./src/client/**/*", "./src/tests/**/*"],
      ...(isContinuousIntegration ? { sourceMap: false } : {}),
    }),
    nodeResolve(),
    args.run
      ? run({
          execArgv: args.inspect ? ["--inspect"] : [],
          args: args.nonRollupArgs,
        })
      : undefined,
  ],
  external(id) {
    return /node_modules/.test(id) || ["tslib", "events", "bindings"].includes(id);
  },
  makeAbsoluteExternalsRelative: false,
};

export default config;

// Adapted from https://github.com/sindresorhus/slash/blob/v4.0.0/index.js
function toUnixPath(windowsPath) {
  const isExtendedLengthPath = /^\\\\\?\\/.test(windowsPath);
  const hasNonAscii = /[^\u0000-\u0080]+/.test(windowsPath); // eslint-disable-line no-control-regex

  if (isExtendedLengthPath || hasNonAscii) {
    return path;
  }

  return windowsPath.replace(/\\/g, "/");
}
