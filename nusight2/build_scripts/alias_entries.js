/* eslint-env node */
// Shared path-alias entries derived from tsconfig.json "paths", for use by both
// Vite (resolve.alias) and Rollup (@rollup/plugin-alias). This keeps the bundler
// alias resolution in sync with the TypeScript path mappings in a single place.
import path from "node:path";
import { fileURLToPath } from "node:url";

import { parseTsconfig } from "get-tsconfig";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const tsconfig = parseTsconfig(path.resolve(__dirname, "../tsconfig.json"));

/** Escape special characters in the given string for use in a regular expression */
const escapeForRegExp = (str) => str.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");

const removeTrailingAsterisk = (str) => str.replace(/\*$/, "");

// Transform the TypeScript path mappings into alias entries that both the Vite
// resolver and the Rollup alias plugin understand ({ find: RegExp, replacement }).
export const aliasEntries = Object.entries(tsconfig?.compilerOptions?.paths ?? {})
  .map(([key, [firstValue]]) => {
    const from = removeTrailingAsterisk(key);
    const to = path.isAbsolute(firstValue)
      ? firstValue
      : path.resolve(__dirname, "..", removeTrailingAsterisk(firstValue));

    return [from, to];
  })
  .map(([key, value]) => {
    return {
      find: new RegExp(`^${escapeForRegExp(key)}(.*)`),
      replacement: `${value}/$1`,
    };
  });
