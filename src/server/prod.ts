import * as compression from 'compression'
import * as express from 'express'
import * as fallback from 'express-history-api-fallback'
import * as http from 'http'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'

const app = express()
const server = http.createServer(app)
sio(server)

const root = `${__dirname}/../../dist`
app.use(compression())
app.use(express.static(root))
app.use(favicon(`${__dirname}/../assets/favicon.ico`))
app.use(fallback('index.html', { root }))

const port = process.env.PORT || 9090
server.listen(port, () => {
  /* tslint:disable no-console */
  console.log(`NUsight server started at http://localhost:${port}`)
})
