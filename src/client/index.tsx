import { useStrict } from 'mobx'
import { runInAction } from 'mobx'
import * as React from 'react'
import * as ReactDOM from 'react-dom'
import { BrowserRouter } from 'react-router-dom'
import { Route } from 'react-router-dom'
import { Switch } from 'react-router-dom'
import { AppController } from './components/app/controller'
import { AppModel } from './components/app/model'
import { AppView } from './components/app/view'
import { Chart } from './components/chart/view'
import { Classifier } from './components/classifier/view'
import { Dashboard } from './components/dashboard/view'
import { GameState } from './components/game_state/view'
import { LocalisationController } from './components/localisation/controller'
import { RobotModel as LocalisationRobotModel } from './components/localisation/darwin_robot/model'
import { LocalisationModel } from './components/localisation/model'
import { LocalisationNetwork } from './components/localisation/network'
import { LocalisationView } from './components/localisation/view'
import { withRobotSelectorMenuBar } from './components/menu_bar/view'
import { NUClear } from './components/nuclear/view'
import { RobotModel } from './components/robot/model'
import { Scatter } from './components/scatter_plot/view'
import { Subsumption } from './components/subsumption/view'
import { Vision } from './components/vision/view'
import { NUsightNetwork } from './network/nusight_network'

// enable MobX strict mode
useStrict(true)

const nusightNetwork = NUsightNetwork.of()
nusightNetwork.connect({ name: 'nusight' })

// TODO (Annable): Replace all this code with real networking + simulator
const localisationModel = LocalisationModel.of()
const appModel = AppModel.of({
  robots: [
    RobotModel.of({ enabled: false, name: 'Robot 1', host: 'localhost' }),
    RobotModel.of({ enabled: false, name: 'Robot 2', host: 'localhost' }),
    RobotModel.of({ enabled: false, name: 'Robot 3', host: 'localhost' }),
  ],
})

runInAction(() => {
  localisationModel.camera.position.set(0, 0.2, 0.5)

  const colors = [undefined, 'magenta', undefined, 'blue', undefined, 'cyan', undefined, 'red']
  const numRobots = 8
  new Array(numRobots).fill(0).map((_, id) => {
    const robot = LocalisationRobotModel.of({ id, name: `Robot ${id + 1}`, color: colors[id] || undefined, heading: 0 })
    localisationModel.robots.push(robot)
    return robot
  })
})

requestAnimationFrame(function update() {
  requestAnimationFrame(update)
  runInAction(() => {
    const numRobots = localisationModel.robots.length
    localisationModel.robots.forEach((robot, i) => {

      const angle = i * (2 * Math.PI) / numRobots + Date.now() / 4E4
      const distance = Math.cos(Date.now() / 1E3 + 4 * i) * 0.3 + 1
      robot.position.x = distance * Math.cos(angle)
      robot.position.z = distance * Math.sin(angle)
      robot.heading = -angle - Math.PI / 2
    })
  })
})

const appController = AppController.of()
const menu = withRobotSelectorMenuBar(appModel.robots, appController.toggleRobotEnabled)

ReactDOM.render(
  <BrowserRouter>
    <AppView>
      <Switch>
        <Route exact path='/' component={Dashboard}/>
        <Route path='/localisation' render={() => {
          const model = localisationModel
          const controller = LocalisationController.of()
          const network = LocalisationNetwork.of(nusightNetwork, model)
          return <LocalisationView controller={controller} menu={menu} model={model} network={network}/>
        }}/>
        <Route path='/vision' component={Vision}/>
        <Route path='/chart' component={Chart}/>
        <Route path='/scatter' component={Scatter}/>
        <Route path='/nuclear' component={NUClear}/>
        <Route path='/classifier' component={Classifier}/>
        <Route path='/subsumption' component={Subsumption}/>
        <Route path='/gamestate' component={GameState}/>
      </Switch>
    </AppView>
  </BrowserRouter>,
  document.getElementById('root'),
)
