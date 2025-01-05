/* eslint-env node */
import * as fs from "fs";
import * as path from "path";

const fileRegex = /\.(vert|frag)$/;

export default function glslPlugin() {
  return {
    name: "transform-glsl",

    async transform(src, id) {
      if (fileRegex.test(id)) {
        const parentDir = path.parse(id).dir;

        return {
          code: await compileSource(src, parentDir, this.addWatchFile.bind(this)),
          map: null,
        };
      }
    },
  };
}

async function compileSource(source, parentDir, addWatchFile) {
  const expandedSource = await expandIncludes(source, {
    parentDir,
    readFile: async (filePath) => {
      // Add the file to the rollup, to watch for changes
      addWatchFile(filePath);

      return await fs.promises.readFile(filePath, "utf-8");
    },
  });

  const json = JSON.stringify(expandedSource)
    // Escape paragraph and line separators which `JSON.stringify()` may allow.
    // Because we're generating JS source, unescaped paragraph and line separators
    // will be intepreted as new lines, which will result in a syntax error.
    // See http://www.thespanner.co.uk/2011/07/25/the-json-specification-is-now-wrong/.
    .replace(/\u2028/g, "\\u2028")
    .replace(/\u2029/g, "\\u2029");

  return `export default ${json};`;
}

const includeRegex = /^[ \t]*#include[ \t]+"([^"\r\n]+)"/gm;

async function replaceAsync(string, pattern, replacer) {
  const replacements = [];

  string.replace(pattern, (...args) => {
    replacements.push(replacer(...args));
    return "";
  });

  return Promise.all(replacements).then((resolvedValues) => {
    return string.replace(pattern, () => {
      return resolvedValues.shift();
    });
  });
}

async function expandIncludes(source, { parentDir, readFile }) {
  return replaceAsync(source, includeRegex, async (_match, includeTarget) => {
    const filePath = path.join(parentDir, includeTarget);
    const includeSource = await readFile(filePath);
    return await expandIncludes(includeSource, { parentDir: path.parse(filePath).dir, readFile });
  });
}
