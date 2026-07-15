import { createEcmaScriptPlugin, runNodeJs, Schema } from "@bufbuild/protoplugin";
import { hashType } from "@shared/nuclearnet/hash_type";

import { getAllTypes } from "./get_types";

const plugin = createEcmaScriptPlugin({
  name: "hash-converter-plugin",
  version: "v1",
  generateTs,
});

function generateTs(schema: Schema) {
  const f = schema.generateFile("hash_converters.ts");
  const messages: string[] = [];
  const types = getAllTypes(schema);

  for (const message of types.messages) {
    if (message.typeName.startsWith("message.")) {
      messages.push(message.typeName);
    }
  }

  const converters = messages.map((message) => {
    const hash = hashType(message).toString("hex");
    return `hashToName.set("${hash}", "${message}");`;
  });

  const file = `
const hashToName = new Map<string, string>();

${converters.join("\n")}

/** Get the fully qualified name of the message from its hash */
export function messageHashToName(hash: string | Buffer): string {
  const hashString = (typeof hash === "string") ? hash : hash.toString("hex");
  const name = hashToName.get(hashString);
  if (name === undefined) {
    throw new Error(\`Could not find name for unknown message hash: \${hashString}\`);
  }
  return name;
}
  `.trim();
  f.print(file);
}

runNodeJs(plugin);
