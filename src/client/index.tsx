import { useStrict } from 'mobx'
import { runInAction } from 'mobx'
import { Provider } from 'mobx-react'
import * as React from 'react'
import * as ReactDOM from 'react-dom'
import { browserHistory, IndexRoute, Route, Router } from 'react-router'
import * as io from 'socket.io-client'
import { AppView } from './components/app/view'
import { Chart } from './components/chart/view'
import { Classifier } from './components/classifier/view'
import { Dashboard } from './components/dashboard/view'
import { GameState } from './components/game_state/view'
import { RobotModel } from './components/localisation/darwin_robot/model'
import { LocalisationModel } from './components/localisation/model'
import { LocalisationPresenter } from './components/localisation/presenter'
import { LocalisationView } from './components/localisation/view'
import { NUClear } from './components/nuclear/view'
import { Scatter } from './components/scatter_plot/view'
import { Subsumption } from './components/subsumption/view'
import { Vision } from './components/vision/view'

// enable MobX strict mode
useStrict(true)

const stores = {
  localisationStore: LocalisationModel.of(),
}

stores.localisationStore.camera.position.set(0, 0.2, 0.5)

const colors = [null, 'magenta', null, 'blue', null, 'cyan', null, 'red']
const numRobots = 8
const robots = new Array(numRobots).fill(0).map((_, id) => {
  const robot = RobotModel.of({ id, name: `Robot ${id + 1}`, color: colors[id], heading: 0 })
  stores.localisationStore.addRobot(robot)
  return robot
})

requestAnimationFrame(function update() {
  requestAnimationFrame(update)
  runInAction(() => {
    robots.forEach((robot, i) => {

      const angle = i * (2 * Math.PI) / numRobots + Date.now() / 4E3
      const distance = Math.cos(Date.now() / 1E3 + 4 * i) * 0.3 + 1
      robot.position.setX(distance * Math.cos(angle))
      robot.position.setZ(distance * Math.sin(angle))
      robot.setHeading(-angle - Math.PI / 2)

      const motorAngle = Math.cos(Date.now() / 1E3 + i) / 2 + 0.5
      robot.motors.rightShoulderPitch.setAngle(motorAngle)
      robot.motors.leftShoulderPitch.setAngle(motorAngle)
      robot.motors.rightShoulderRoll.setAngle(motorAngle)
      robot.motors.leftShoulderRoll.setAngle(motorAngle)
      robot.motors.rightElbow.setAngle(motorAngle)
      robot.motors.leftElbow.setAngle(motorAngle)
      robot.motors.rightHipYaw.setAngle(motorAngle)
      robot.motors.leftHipYaw.setAngle(motorAngle)
      robot.motors.rightHipRoll.setAngle(motorAngle)
      robot.motors.leftHipRoll.setAngle(motorAngle)
      robot.motors.rightHipPitch.setAngle(motorAngle)
      robot.motors.leftHipPitch.setAngle(motorAngle)
      robot.motors.rightKnee.setAngle(motorAngle)
      robot.motors.leftKnee.setAngle(motorAngle)
      robot.motors.rightAnklePitch.setAngle(motorAngle)
      robot.motors.leftAnklePitch.setAngle(motorAngle)
      robot.motors.rightAnkleRoll.setAngle(motorAngle)
      robot.motors.leftAnkleRoll.setAngle(motorAngle)
      // robot.motors.headPan.setAngle(angle)
      // robot.motors.headTilt.setAngle(angle)
    })
  })
})

io.connect(document.location.origin)

// render react DOM
ReactDOM.render(
    <Provider {...stores} >
      <Router history={browserHistory}>
        <Route path='/' component={AppView}>
          <IndexRoute component={Dashboard}/>
          <Route path='/localisation' component={() => {
            const presenter = LocalisationPresenter.of({
              model: stores.localisationStore,
            })
            return <LocalisationView presenter={presenter} localisationStore={stores.localisationStore}/>
          }}/>
          <Route path='/vision' component={Vision}/>
          <Route path='/chart' component={Chart}/>
          <Route path='/scatter' component={Scatter}/>
          <Route path='/nuclear' component={NUClear}/>
          <Route path='/classifier' component={Classifier}/>
          <Route path='/subsumption' component={Subsumption}/>
          <Route path='/gamestate' component={GameState}/>
        </Route>
      </Router>
    </Provider >,
    document.getElementById('root'),
)
