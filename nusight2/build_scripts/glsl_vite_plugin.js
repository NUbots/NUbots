const fs = require('fs')
const path = require('path')
const { loadGlsl } = require('./glsl_loader')

export default function glslPlugin() {
  const glslFileRegex = /\.(vert|frag)$/

  return {
    name: 'transform-glsl',

    async transform(src, id) {
      if (glslFileRegex.test(id)) {
        const parentDir = path.parse(id).dir

        return {
          code: await loadGlsl(src, {
            parentDir,
            readFile: async filePath => {
              // Add the file to rollup, to watch for changes
              this.addWatchFile(filePath)
              return await fs.promises.readFile(filePath, 'utf-8')
            },
          }),
          map: null,
        }
      }
    },
  }
}
