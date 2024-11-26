/* eslint-env node */
import * as fs from "fs";
import { createRequire } from 'module';
import * as path from "path";
const require = createRequire(import.meta.url);
const pbjs = require("protobufjs-cli/pbjs");
const pbts = require("protobufjs-cli/pbts");

const __dirname = import.meta.dirname;

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const NUSIGHT_ROOT = path.resolve(__dirname, "..");

const rootDir = path.resolve(REPO_ROOT, "shared");
const messagesDir = path.resolve(REPO_ROOT, "shared/message");
const nuclearMessagesDir = path.resolve(REPO_ROOT, "nuclear/message/proto");
const googleMessagesDir = path.resolve(NUSIGHT_ROOT, "src/shared/messages/proto");

const messageSourceDirs = [rootDir, messagesDir, nuclearMessagesDir, googleMessagesDir];

const messageSourceFiles = [
  `${messagesDir}/**/*.proto`,
  `${nuclearMessagesDir}/**/*.proto`,
  `${googleMessagesDir}/**/*.proto`,
].map((f) => f.replaceAll(path.win32.sep, path.posix.sep)); // pbjs expects posix directory separators.

// Data types to protobuf wire types
const protobufWireTypes = {
  // Varint fields
  int32: 0,
  int64: 0,
  uint32: 0,
  uint64: 0,
  sint32: 0,
  sint64: 0,
  bool: 0,
  enum: 0,

  // 64-bit fields
  fixed64: 1,
  sfixed64: 1,
  double: 1,

  // Length-delimited fields
  string: 2,
  bytes: 2,

  // Note: wire types for composite fields (i.e. embedded messages, groups, packed repeated fields)
  // are omitted from this list

  // 32-bit fields
  fixed32: 5,
  sfixed32: 5,
  float: 5,
};

/**
 * The protobuf.js JSON description object has the following shape:
 *    {
 *      "nested": {
 *        "namespaceA": {
 *          "nested": {
 *            "namespaceB": {
 *              "nested": {
 *                "MessageTypeA": {
 *                  "fields": {
 *                    "fieldA": {
 *                      "type": "double",
 *                      "id": 1
 *                    },
 *                    "fieldB": {
 *                      "type": "uint32",
 *                      "id": 2
 *                    }
 *                  },
 *                  "nested": {
 *                    "SubMessageA": {...},
 *                    "SubMessageB": {...}
 *                  }
 *                },
 *                "MessageTypeB": {...}
 *              }
 *            }
 *          }
 *        }
 *      }
 *    }
 *
 * This function walks the description and calls the `processNode()` function for each message node.
 */
function walkMessageFields(messageFieldsDesc, processNode) {
  walkMessageFieldsRecursive(messageFieldsDesc, processNode, { parentKey: "", messagePath: "" });
}

function walkMessageFieldsRecursive(descNode, processNode, { parentKey, messagePath }) {
  for (const [nodeKey, nodeValue] of Object.entries(descNode)) {
    // If the key is 'fields', we've arrived at the fields for a message
    if (nodeKey === "fields") {
      processNode({ key: nodeKey, value: nodeValue, path: messagePath, parentKey });
    }

    // We've arrived at a node that has nested nodes. This could be a namespace,
    // or a message that has sub messages: process it recursively.
    else if (nodeKey === "nested") {
      walkMessageFieldsRecursive(nodeValue, processNode, { messagePath, parentKey: nodeKey });
    }

    // The description node we're processing came from the key 'nested', so it will have
    // messages as its direct children. Process them here recursively, appending the
    // current node's key to the message path.
    else if (parentKey === "nested") {
      walkMessageFieldsRecursive(nodeValue, processNode, {
        messagePath: messagePath.length === 0 ? nodeKey : `${messagePath}.${nodeKey}`,
        parentKey: nodeKey,
      });
    }
  }
}

function generateMessagesJs(outputFilePath) {
  // prettier-ignore
  const args = [
    '--target', 'static-module',
    '--es6',
    // Use a custom wrapper, as the builtin es6 wrapper produces invalid output
    // Refer to https://github.com/protobufjs/protobuf.js/issues/1862#issuecomment-1660014799
    '--wrap', path.join(__dirname, 'proto_wrapper.js'),
    '--out', outputFilePath,
    '--no-create', '--no-verify', '--no-convert', '--no-delimited',
    ...messageSourceDirs.map(dir => ['--path', dir]).flat(),
    ...messageSourceFiles
  ]

  return new Promise((resolve, reject) => {
    pbjs.main(args, (err) => {
      if (err) {
        reject(err);
      } else {
        console.log(`Generated messages JS: ${outputFilePath}`);
        resolve();
      }
    });
  });
}

function generateMessagesTs(inputFilePath, outputFilePath) {
  const generateTs = () => {
    return new Promise((resolve, reject) => {
      // prettier-ignore
      const args = [
      inputFilePath,
      '--out', outputFilePath,
      '--name', '',
    ]

      pbts.main(args, (err) => {
        if (err) {
          reject(err);
        } else {
          resolve();
        }
      });
    });
  };

  const appendCustomTypes = () => {
    const customTypes = `
// NUsight-specific additions to the Protobuf.js generated types ---------------

/** Represents a Protobuf.js message class of type T */
export interface MessageType<T> {
  new (...args: any[]): T;
  encode(message: T, writer?: $protobuf.Writer): $protobuf.Writer;
  decode(reader: $protobuf.Reader | Uint8Array, length?: number): T;
}`.trim();

    return fs.promises.appendFile(outputFilePath, "\n\n" + customTypes);
  };

  return generateTs()
    .then(appendCustomTypes)
    .then(() => {
      console.log(`Generated messages TS: ${outputFilePath}`);
    });
}

function generateMessageFields() {
  // prettier-ignore
  const args = [
    '--target', 'json',
    ...messageSourceDirs.map(dir => ['--path', dir]).flat(),
    ...messageSourceFiles
  ]

  return new Promise((resolve, reject) => {
    pbjs.main(args, function (err, output) {
      if (err) {
        reject(err);
      } else {
        resolve(JSON.parse(output));
      }
    });
  });
}

async function generateMessageFieldsIndex(outputFilePath, messageFieldsDesc) {
  const index = {};
  const fieldsToIndex = ["id"];

  walkMessageFields(messageFieldsDesc, (node) => {
    for (const [fieldName, fieldDesc] of Object.entries(node.value)) {
      // Skip to the next field if this field is not in the list of fields to index
      if (!fieldsToIndex.includes(fieldName)) {
        continue;
      }
      // Otherwise add the field to the message it belongs to in the index
      else {
        const indexEntry = index[node.path] ?? {};

        indexEntry[fieldName] = fieldDesc;
        indexEntry[fieldName].wireType = protobufWireTypes[fieldDesc.type] ?? -1;

        index[node.path] = indexEntry;
      }
    }
  });

  const types = `
export interface MessageField {
  wireType: number
  type: string
  id: number
}

export interface MessageFieldsIndex {
  [messageName: string]: {
    [fieldName: string]: MessageField
  }
}
  `.trim();

  const file =
    types + "\n\n" + "export const messageFieldsIndex: MessageFieldsIndex = " + JSON.stringify(index, null, "  ");

  await fs.promises.writeFile(outputFilePath, file);

  console.log(`Generated message fields index: ${outputFilePath}`);
}

async function generateRpcCallTypes(outputFilePath, messageFieldsDesc) {
  const rpcMessages = [];

  walkMessageFields(messageFieldsDesc, (node) => {
    const isRpcRequestMessage =
      node.parentKey.endsWith("Request") &&
      Array.from(Object.keys(node.value)).some((fieldName) => fieldName === "rpc");

    if (isRpcRequestMessage) {
      rpcMessages.push({
        request: node.path,
        response: node.path + ".Response",
      });
    }
  });

  const callOverloads = rpcMessages.map(({ request, response }) => {
    return `function rpcCall<T = ${response}>(request: ${request}, options?: RpcCallOptions): Promise<RpcResult<T>>;`;
  });

  const file = `
import type { message } from "../../shared/messages";
import type { RobotModel } from "../components/robot/model";

export type RpcErrorCause = "INVALID_REQUEST" | "TIMEOUT" | "CANCELLED" | "REMOTE_ERROR" | "UNKNOWN";

export interface RpcErrorOptions {
  cause: RpcErrorCause;
  request: any;
  RequestType: any;
  ResponseType: any;
}

export class RpcError extends Error {
  cause: RpcErrorOptions["cause"];
  request: RpcErrorOptions["request"];
  RequestType: RpcErrorOptions["RequestType"];
  ResponseType: RpcErrorOptions["ResponseType"];
  constructor(message: string, options: RpcErrorOptions);
  /** Determine if the error was caused by an error on the remote end */
  isRemoteError(): boolean;
  /** Determine if the error was caused by the request timing out */
  isTimeout(): boolean;
  /** Determine if the error was caused by the request being cancelled */
  isCancelled(): boolean;
  /** Handle the error based on its cause. Ignores errors caused by cancellation, and throws all other errors. */
  defaultHandler(): void;
}

export interface RpcCallOptions { target?: string; timeout?: number; }

export interface RpcResultData<T> { robotModel: RobotModel; response: T; }

export type RpcResult<T> = { ok: true; data: RpcResultData<T> } | { ok: false; error: RpcError };

${callOverloads.join("\n")}

export type RpcCall = typeof rpcCall;
`.trim();

  await fs.promises.writeFile(outputFilePath, file);

  console.log(`Generated RPC call types: ${outputFilePath}`);
}

async function generateEmitTypes(outputFilePath, messageFieldsDesc) {
  const messages = [];

  walkMessageFields(messageFieldsDesc, (node) => {
    if (node.path.startsWith("message.")) {
      messages.push(node.path);
    }
  });

  const emitOverloads = messages.map((message) => {
    return `function emit(message: ${message}, options?: EmitOptions): void;`;
  });

  const file = `
import type { message } from "./index";

export interface EmitOptions { target?: string; reliable?: boolean; }

${emitOverloads.join("\n")}

export type Emit = typeof emit;
`.trim();

  await fs.promises.writeFile(outputFilePath, file);

  console.log(`Generated emit types: ${outputFilePath}`);
}

async function generateMessageTypeConverters(outputFilePath, messageFieldsDesc) {
  const messages = [];

  walkMessageFields(messageFieldsDesc, (node) => {
    if (node.path.startsWith("message.")) {
      messages.push(node.path);
    }
  });

  const converters = messages.map((message) => {
    return [`nameToType.set("${message}", ${message});`, `typeToName.set(${message}, "${message}");`].join("\n");
  });

  const file = `
import { message, MessageType } from "./index";

const nameToType = new Map<string, any>();
const typeToName = new Map<any, string>();

${converters.join("\n\n")}

/** Get the fully qualified name of a message type (Protobuf class) */
export function messageTypeToName(type: MessageType<any>): string {
  const name = typeToName.get(type);
  if (name === undefined) {
    throw new Error(\`Could not find name for unknown message type: \${type}\`);
  }
  return name;
}

/** Get the message type (Protobuf class) for a fully qualified message name */
export function messageNameToType<T = any>(name: string): MessageType<T> {
  const type = nameToType.get(name);
  if (type === undefined) {
    throw new Error(\`Could not find type for unknown message name: \${name}\`);
  }
  return type;
}
`.trim();

  await fs.promises.writeFile(outputFilePath, file);

  console.log(`Generated message type converters: ${outputFilePath}`);
}

async function main() {
  const messagesOutputDir = path.resolve(NUSIGHT_ROOT, "src/shared/messages");

  const messagesJsFile = path.join(messagesOutputDir, "index.js");
  const messagesTsFile = path.join(messagesOutputDir, "index.d.ts");
  const messagesIndexFile = path.join(messagesOutputDir, "fields_index.ts");
  const messageTypeConvertersFile = path.join(messagesOutputDir, "type_converters.ts");
  const emitTypesFile = path.join(messagesOutputDir, "emit.d.ts");
  const rpcCallTypesFile = path.join(messagesOutputDir, "rpc_call.d.ts");

  generateMessagesJs(messagesJsFile).then(() => {
    generateMessagesTs(messagesJsFile, messagesTsFile);
  });

  const messageFieldsDesc = await generateMessageFields();

  await Promise.all([
    generateMessageFieldsIndex(messagesIndexFile, messageFieldsDesc),
    generateMessageTypeConverters(messageTypeConvertersFile, messageFieldsDesc),
    generateRpcCallTypes(rpcCallTypesFile, messageFieldsDesc),
    generateEmitTypes(emitTypesFile, messageFieldsDesc),
  ]);
}

main();
