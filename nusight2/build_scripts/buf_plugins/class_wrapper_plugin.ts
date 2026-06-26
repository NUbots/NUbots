import { createEcmaScriptPlugin, runNodeJs, Schema } from "@bufbuild/protoplugin";

import { getTypesGroupedByLocation } from "./get_types";

const plugin = createEcmaScriptPlugin({
  name: "class-wrapper-plugin",
  version: "v1",
  generateTs,
});

function generateTs(schema: Schema) {
  const typesGroupedByLocation = getTypesGroupedByLocation(schema);

  typesGroupedByLocation.forEach((group) => {
    const f = schema.generateFile(`${group.location}.ts`);

    const imports: string[] = [];
    const interfaceExports: string[] = [];
    const enumExports: string[] = [];
    const enumConverters: string[] = [];
    const classes: string[] = [];

    // Generate imports, interface and class for each message
    group.messages.forEach((message) => {
      imports.push(`type ${message.name} as I${message.name}, ${message.name}Schema`);
      interfaceExports.push(`I${message.name}`);

      // Generate the class wrapper. The exported interface with the same name is used to make
      // the class inherit the properties and documentation comments from the interface.
      const wrapperClass = `export interface ${message.name} extends I${message.name} {};

export class ${message.name} {
  static typeName = ${message.name}Schema.typeName as I${message.name}["$typeName"];
  static schema = ${message.name}Schema;

  constructor(data: MessageInitShape<typeof ${message.name}Schema>) {
    Object.assign(this, create(${message.name}Schema, data));
  }

  toBinary() {
    return toBinary(${message.name}Schema, this);
  }

  static fromBinary(data: Uint8Array) {
    return new ${message.name}(fromBinary(${message.name}Schema, data));
  }
}`;
      classes.push(wrapperClass);
    });

    // Generate imports, a union type, and converters from enum to union type and vice versa
    group.enums.forEach((enumVar) => {
      imports.push(`${enumVar.name} as ${enumVar.name}Enum`);
      enumExports.push(enumVar.name + "Enum");

      const enumUnionTypeAndConverters = `
export type ${enumVar.name} = ${Object.keys(enumVar.values)
        .map((c) => `"${c.toLocaleLowerCase().replaceAll("_", "-")}"`)
        .join(" | ")};

export const ${enumVar.name + "FromEnum"} : Record<${enumVar.name + "Enum"}, ${enumVar.name}> = {
  ${Object.keys(enumVar.values)
    .map((key) => `[${enumVar.name}Enum.${key}]: "${key.toLowerCase().replaceAll("_", "-")}"`)
    .join(",\n  ")}
} as const;

export const ${enumVar.name + "ToEnum"} : Record<${enumVar.name}, ${enumVar.name + "Enum"}> = {
  ${Object.keys(enumVar.values)
    .map((key) => `"${key.toLowerCase().replaceAll("_", "-")}" : ${enumVar.name}Enum.${key}`)
    .join(",\n  ")}
} as const;
        `.trim();

      enumConverters.push(enumUnionTypeAndConverters);
    });

    // If there are messages/enum classes, add the imports from the protobuf library that they use
    if (group.messages.length > 0) {
      f.print(`/* eslint-disable simple-import-sort/imports */\n`);
      f.print(`import { create, fromBinary, MessageInitShape, toBinary } from "@bufbuild/protobuf";`);
    }

    // Import the types from the generated protobuf file
    f.print(`import { ${imports.join(", ")} } from "./${group.fileName}_pb";`);

    // Export the `I` prefixed interfaces
    if (interfaceExports.length > 0) {
      f.print();
      f.print(`export { ${interfaceExports.join(", ")} };`);
    }

    // Export the native enums, with "Enum" suffixed
    if (enumExports.length > 0) {
      f.print();
      f.print(`export { ${enumExports.join(", ")} };`);
    }

    // Export the enum union types and converters
    if (enumConverters.length > 0) {
      f.print();
      f.print(enumConverters.join("\n\n"));
    }

    // Export the message wrapper classes
    if (classes.length > 0) {
      f.print();
      f.print(classes.join("\n\n"));
    }
  });
}

runNodeJs(plugin);
