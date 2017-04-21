import * as compression from 'compression'
import * as express from 'express'
import * as fallback from 'express-history-api-fallback'
import * as http from 'http'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'
import * as webpack from 'webpack'
import * as webpackDevMiddleware from 'webpack-dev-middleware'
import * as webpackHotMiddleware from 'webpack-hot-middleware'
import webpackConfig from '../../webpack.config'
const compiler = webpack(webpackConfig)

const app = express()
const server = http.createServer(app)
sio(server)

app.use(compression())
app.use(webpackDevMiddleware(compiler, {
  index: 'index.html',
  stats: {
    colors: true,
  },
}))
app.use(webpackHotMiddleware(compiler))
const root = `${__dirname}/../../`
app.use(favicon(`${__dirname}/../assets/favicon.ico`))
// TODO (Annable): Fix this clobbering SourceMap URLs.
app.use(fallback('dist/index.html', { root }))

const port = process.env.PORT || 3000
server.listen(port, () => {
  /* tslint:disable no-console */
  console.log(`NUsight server started at http://localhost:${port}`)
})
