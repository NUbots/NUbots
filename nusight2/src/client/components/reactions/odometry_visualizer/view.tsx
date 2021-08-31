import React from 'react'
import { ReactionStats } from '../model'
import styles from './styles.css'

export const ReactionVisualizer = ({ stats }: { stats: ReactionStats }) => {
  return (
    <div className={styles.visualizer}>
      <div>{stats.name}</div>
      <div>{stats.triggerName}</div>
      <div>{stats.functionName}</div>
      <div>{stats.reactionId}</div>
      <div>{stats.taskId}</div>
      <div>{stats.causeReactionId}</div>
      <div>{stats.causeTaskId}</div>
      <div>{stats.emitted}</div>
      <div>{stats.started}</div>
      <div>{stats.finished}</div>
    </div>
  )
}
