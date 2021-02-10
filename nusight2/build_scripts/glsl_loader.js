const fs = require('fs')
const path = require('path')

const includeRegex = /^[ \t]*#include[ \t]+"([^"\r\n]+)"/gm

async function replaceAsync(string, pattern, replacer) {
  const replacements = []

  string.replace(pattern, (...args) => {
    replacements.push(replacer.apply(undefined, args))
    return ''
  })

  return Promise.all(replacements).then(resolvedValues => {
    return string.replace(pattern, () => {
      return resolvedValues.shift()
    })
  })
}

async function expandIncludes(source, { parentDir, readFile }) {
  return replaceAsync(source, includeRegex, async (_match, includeTarget) => {
    const filePath = path.join(parentDir, includeTarget)
    const includeSource = await readFile(filePath)
    return await expandIncludes(includeSource, { parentDir: path.parse(filePath).dir, readFile })
  })
}

export default function glslLoader(source) {
  const callback = this.async()

  expandIncludes(source, {
    parentDir: this.context,
    readFile: async filePath => {
      // add the file to the webpack dependency graph, to watch for changes
      this.addDependency(filePath)

      try {
        return await fs.promises.readFile(filePath, 'utf-8')
      } catch (error) {
        callback(error)
      }
    },
  }).then(expandedSource => {
    const json = JSON.stringify(expandedSource)
      // Escape paragraph and line separators which `JSON.stringify()` may allow.
      // Because we're generating JS source, unescaped paragraph and line separators
      // will be intepreted as new lines, which will result in a syntax error.
      // See http://www.thespanner.co.uk/2011/07/25/the-json-specification-is-now-wrong/.
      .replace(/\u2028/g, '\\u2028')
      .replace(/\u2029/g, '\\u2029')

    callback(null, `export default ${json};`)
  })
}
