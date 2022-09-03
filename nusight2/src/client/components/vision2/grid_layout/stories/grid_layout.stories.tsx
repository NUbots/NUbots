import { number } from '@storybook/addon-knobs'
import { withKnobs } from '@storybook/addon-knobs'
import { storiesOf } from '@storybook/react'
import { observable } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { SeededRandom } from '../../../../../shared/base/random/seeded_random'
import { range } from '../../../../../shared/base/range'
import { fullscreen } from '../../../storybook/fullscreen'
import images from '../../image_view/stories/images/image.jpg'
import { GridLayout } from '../grid_layout'
import styles from './styles.css'
import useInterval from '@use-it/interval'

const imageWidth = 600
const imageHeight = 480
const aspectRatio = imageHeight / imageWidth

storiesOf('components.vision.grid_layout', module)
  .addDecorator(fullscreen)
  .addDecorator(withKnobs)
  .add('Renders static', () => {
    const numItems = number('Items', 2)
    const subItems = number('Sub-items', 1)
    return (
      <GridLayout itemAspectRatio={aspectRatio}>
        {range(numItems).map(i => (
          <div key={i} className={styles.item} style={{ backgroundColor: 'red' }}>
            <GridLayout itemAspectRatio={aspectRatio}>
              {range(subItems).map(i => (
                <div key={i} className={styles.item}>
                  <img className={styles.image} src={images} style={{ objectFit: 'contain' }} />
                </div>
              ))}
            </GridLayout>
          </div>
        ))}
      </GridLayout>
    )
  })
  .add('Renders animated', () => {
    const numItems = number('Items', 2)
    const subItems = number('Sub-items', 1)
    const width = observable.box(1)
    const random = SeededRandom.of('grid_layout')
    const Component = observer(() => {
      useInterval(() => width.set(random.float()), 1000)
      return (
        <div
          style={{
            height: '100%',
            width: `${width.get() * 100}%`,
            transition: 'width 500ms ease',
            border: '1px solid black',
            boxSizing: 'border-box',
          }}
        >
          <GridLayout itemAspectRatio={aspectRatio}>
            {range(numItems).map(i => (
              <div key={i} className={styles.item} style={{ backgroundColor: 'red' }}>
                <GridLayout itemAspectRatio={aspectRatio}>
                  {range(subItems).map(i => (
                    <div key={i} className={styles.item}>
                      <img className={styles.image} src={images} style={{ objectFit: 'contain' }} />
                    </div>
                  ))}
                </GridLayout>
              </div>
            ))}
          </GridLayout>
        </div>
      )
    })
    return <Component />
  })
