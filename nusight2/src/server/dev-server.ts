import minimist from 'minimist'
import nodemon from 'nodemon'
import * as path from 'path'
import webpack from 'webpack'

import { getServerConfig } from '../../webpack.config'

// This dev server achieves the following:
//
// 1. Both server-side and client-side code are compiled via webpack (allowing e.g. import loaders in both).
// 2. Any changes to client-side code are recompiled, without restarting the node server.
// 3. Any changes to server-side code are recompiled, restarting the node server (which recompiles the client-side).
//
// This is made possible by two separate webpack compiler instances, in two separate processes:
// 1. The compiler instance in this file handles the server-side.
// 2. The compiler instance spawned by nodemon handles the client-side.
//
// Both instances listen for updates, and restart and recompile appropriately.

const args = minimist(process.argv.slice(2))
const compiler = webpack(
  getServerConfig({
    mode: 'development',
    context: path.join(__dirname, '..'),
    transpileOnly: args.t || args.transpileOnly || false,
    rootDir: path.join(__dirname, '..', '..'),
  }),
)

let startedServer = false
compiler.watch({}, (err, stats) => {
  if (err) {
    process.stderr.write(err + '\n')
    return
  }
  stats && process.stdout.write(stats.toString({ colors: true }) + '\n')

  if (!startedServer) {
    nodemon({
      script: 'dist/dev.js',
      exec: 'node',
      args: ['--context', path.join(__dirname, '..'), ...process.argv.slice(2)],
      watch: ['dist/dev.js'],
      ext: 'js',
    })
      // tslint:disable-next-line no-console
      .on('start', () => console.log('Starting server'))
      // tslint:disable-next-line no-console
      .on('restart', () => console.log('Changes detected, restarting server'))
      .on('quit', () => process.exit())
    startedServer = true
  }
})
