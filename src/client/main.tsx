import { configure } from 'mobx'
import React from 'react'
import ReactDOM from 'react-dom'

import AppView from './components/app/view'

configure({ enforceActions: 'observed' })
ReactDOM.render(<AppView/>, document.getElementById('root'))
