const fs = require('fs')
const { loadGlsl } = require('./glsl_loader')

export default function glslLoader(source) {
  const callback = this.async()

  loadGlsl(source, {
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
  }).then(glslSource => {
    callback(null, glslSource)
  })
}
