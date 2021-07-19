import React from 'react'
import styles from './styles.css'

export type ReactionStats = {
     name               :string;
     trigger_name       :string;
     function_name      :string;
     reaction_id        :number;
     task_id            :number;
     cause_reaction_id  :number;
     cause_task_id      :number;
     emitted            :number;
     started            :number;
     finished           :number;
}

export const OdometryVisualizer = ( stats: ReactionStats ) => {


    return (
      <div className={styles.visualizer}>
        <div>
            {stats.name}
        </div>
        <div>
            {stats.trigger_name}
        </div>
        <div>
            {stats.function_name}
        </div>
        <div>
            {stats.reaction_id}
        </div>
        <div>
            {stats.task_id}
        </div>
        <div>
            {stats.cause_reaction_id}
        </div>
        <div>
            {stats.cause_task_id}
        </div>
        <div>
            {stats.emitted}
        </div>
        <div>
            {stats.started}
        </div>
        <div>
            {stats.finished}
        </div>

      </div>
    )


}
