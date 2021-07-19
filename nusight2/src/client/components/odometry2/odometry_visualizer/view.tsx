import React from 'react'
import { OdometryVisualizerModel } from './model'
import styles from './styles.css'

export type ReactionStatsProps = {
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

export class OdometryVisualizer extends React.Component<{ model: OdometryVisualizerModel }> {

  render() {
    return (
      <div className={styles.visualizer}>
        <div className={styles.legend}>
          <div className={styles.item}>
            Oh hi there
          </div>
        </div>
      </div>
    )
  }

}
