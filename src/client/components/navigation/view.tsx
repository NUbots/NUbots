import * as React from 'react'
import { IndexLink, Link as NormalLink } from 'react-router'
import ChartIcon from './icons/chart.svg'
import ControllerIcon from './icons/controller.svg'
import CubeIcon from './icons/cube.svg'
import EyeIcon from './icons/eye.svg'
import MapIcon from './icons/map.svg'
import NUClearIcon from './icons/nuclear.svg'
import OrderingIcon from './icons/ordering.svg'
import ScatterIcon from './icons/scatter.svg'
import SpeedometerIcon from './icons/speedometer.svg'
import * as style from './style.css'

const NavigationItemView = ({ url, Icon, children = undefined, Link = NormalLink }) => (
    <li className={style.header__item}>
      <Link className={style.header__link} to={url} activeClassName={style['header__link--active']}>
        <Icon className={style.header__icon}/>
        <span>{children}</span>
      </Link>
    </li>
)

export const NavigationView = () => (
    <header className={style.header}>
      <div className={style.header__logoContainer}>
        <img src='images/nusight.jpg' className={style.header__logo}/>
        <h1 className={style.header__title}>NUsight2</h1>
      </div>
      <ul className={style.header__list}>
        <NavigationItemView url='/' Icon={SpeedometerIcon} Link={IndexLink}>Dashboard</NavigationItemView>
        <NavigationItemView url='/localisation' Icon={MapIcon}>Localisation</NavigationItemView>
        <NavigationItemView url='/vision' Icon={EyeIcon}>Vision</NavigationItemView>
        <NavigationItemView url='/chart' Icon={ChartIcon}>Chart</NavigationItemView>
        <NavigationItemView url='/scatter' Icon={ScatterIcon}>Scatter</NavigationItemView>
        <NavigationItemView url='/nuclear' Icon={NUClearIcon}>NUClear</NavigationItemView>
        <NavigationItemView url='/classifier' Icon={CubeIcon}>Classifier</NavigationItemView>
        <NavigationItemView url='/subsumption' Icon={OrderingIcon}>Subsumption</NavigationItemView>
        <NavigationItemView url='/gamestate' Icon={ControllerIcon}>GameState</NavigationItemView>
      </ul>
    </header>
)

export default NavigationView
