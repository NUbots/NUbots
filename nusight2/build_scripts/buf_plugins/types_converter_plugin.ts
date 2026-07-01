import { createEcmaScriptPlugin, runNodeJs, Schema } from "@bufbuild/protoplugin";

import { getTypesGroupedByLocation } from "./get_types";

const plugin = createEcmaScriptPlugin({
  name: "message-types-plugin",
  version: "v1",
  generateTs,
});

function generateTs(schema: Schema) {
  const f = schema.generateFile("type_converters.ts");
  const typesGroupedByLocation = getTypesGroupedByLocation(schema);

  const messageTypeMap: string[] = [];
  const imports: string[] = [];

  typesGroupedByLocation.forEach((group) => {
    if (group.messages.some((message) => message.typeName.startsWith("message."))) {
      messageTypeMap.push(
        group.messages
          .map((message) => [`nameToType.set("${message.typeName}", ${message.typeName.replaceAll(".", "_")});`])
          .join("\n"),
      );
      imports.push(
        `import { ${group.messages.map((message) => `${message.name} as ${message.typeName.replaceAll(".", "_")}`).join(", ")} } from "@proto/${group.location}";`,
      );
    }
  });

  const file = `
/* eslint-disable simple-import-sort/imports */

import { MessageInstance, MessageType } from "../types";
${imports.join("\n")}

const nameToType = new Map<string, any>();

${messageTypeMap.join("\n")}

/** Get the message type (Protobuf class) with the given fully qualified message name */
export function messageNameToType<T extends MessageInstance>(name: string): MessageType<T> {
  const type = nameToType.get(name);
  if (type === undefined) {
    throw new Error(\`Could not find type for unknown message name: \${name}\`);
  }
  return type;
}
  `.trim();
  f.print(file);
}

runNodeJs(plugin);
