import { configure } from 'mobx'
import * as React from 'react'
import * as ReactDOM from 'react-dom'

import AppView from './components/app/view'

configure({ enforceActions: 'observed' })
ReactDOM.render(<AppView/>, document.getElementById('root'))
