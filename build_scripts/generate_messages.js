const path = require('path')
const pbjs = require('protobufjs/cli/pbjs')
const pbts = require('protobufjs/cli/pbts')

const messagesDir = path.resolve(__dirname, '..', 'src/shared/proto')
const messagesFileJs = path.join(messagesDir, 'messages.js')
const messagesFileTs = path.join(messagesDir, 'messages.d.ts')

const argsJs = [
  '--target', 'static-module',
  '--wrap', 'commonjs',
  '--out', messagesFileJs,
  '--path', messagesDir,
  `${messagesDir}/**/*.proto`,
]

const argsTs = [
  messagesFileJs,
  '--out', messagesFileTs,
  '--name',
]

pbjs.main(argsJs, (err, output) => {
  if (err) {
    console.error(err)
  } else {
    console.log(`Generated JS: ${messagesFileJs}`)

    pbts.main(argsTs, (err, output) => {
      if (err) {
        console.error(err)
      } else {
        console.log(`Generated TS: ${messagesFileTs}`)
      }
    })
  }
})
