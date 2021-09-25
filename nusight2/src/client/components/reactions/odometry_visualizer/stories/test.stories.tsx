import { storiesOf } from '@storybook/react'
import React from 'react'
import { RobotModel } from '../../../robot/model'
import { fullscreen } from '../../../storybook/fullscreen'
import { ReactionStats } from '../../model'
import { ReactionVisualizerModel } from '../model'
import { ReactionVisualizer } from '../view'

storiesOf('components.reactions.reaction_visualizer', module)
  .addDecorator(fullscreen)
  .add('Renders statically', () => <ReactionVisualizerHarness />)

class ReactionVisualizerHarness extends React.Component<{ animate?: boolean }> {
  const reaction : ReactionStats = getReaction();
  private model = <ReactionVisualizerModel reaction={this.reaction/>

  render() {
      return <ReactionVisualizer model={this.model} />
    }
  }

}
function getReaction(): ReactionStats {
  return {
    name: 'a',
    triggerName: 'string',
    functionName: 'string',
    reactionId: 1,
    taskId: 1,
    causeReactionId:1,
    causeTaskId: 1,
    emitted: 1,
    started: 1,
    finished: 1,
  }

}

function getRobots(): RobotModel[] {
  return [
    {
      id: '1',
      name: 'Virtual Robot 1',
      connected: true,
      enabled: true,
      address: '10.222.10.14',
      port: 57619,
    },
    {
      id: '2',
      name: 'Virtual Robot 2',
      connected: true,
      enabled: true,
      address: '10.222.10.19',
      port: 36261,
    },
    {
      id: '3',
      name: 'Virtual Robot 3',
      connected: false,
      enabled: true,
      address: '10.222.10.20',
      port: 67341,
    },
  ]
}
