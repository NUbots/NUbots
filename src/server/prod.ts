import * as compression from 'compression'
import * as history from 'connect-history-api-fallback'
import * as express from 'express'
import * as http from 'http'
import * as favicon from 'serve-favicon'
import * as sio from 'socket.io'

const app = express()
const server = http.createServer(app)
sio(server)

const root = `${__dirname}/../../build`
app.use(history())
app.use(compression())
app.use(express.static(root))
app.use(favicon(`${__dirname}/../assets/favicon.ico`))

const port = process.env.PORT || 9090
server.listen(port, () => {
  /* tslint:disable no-console */
  console.log(`NUsight server started at http://localhost:${port}`)
})
