const path = require('path')
const pbjs = require('protobufjs/cli/pbjs')
const pbts = require('protobufjs/cli/pbts')

const REPO_ROOT = path.resolve(__dirname, '..', '..');
const NUSIGHT_ROOT = path.resolve(__dirname, '..');

const messagesDir = path.join(REPO_ROOT, 'shared/message')
const nuclearMessagesDir = path.join(REPO_ROOT, 'nuclear/message/proto')
const googleMessagesDir = path.join(NUSIGHT_ROOT, 'src/shared/proto')

const messagesOutputDir = path.join(NUSIGHT_ROOT, 'src/shared')
const messagesFileJs = path.join(messagesOutputDir, 'messages.js')
const messagesFileTs = path.join(messagesOutputDir, 'messages.d.ts')

const argsJs = [
  '--target', 'static-module',
  '--wrap', 'commonjs',
  '--out', messagesFileJs,
  '--path', messagesDir,
  '--path', nuclearMessagesDir,
  '--path', googleMessagesDir,
  `${messagesDir}/**/*.proto`,
  `${nuclearMessagesDir}/**/*.proto`,
  `${googleMessagesDir}/**/*.proto`,
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
