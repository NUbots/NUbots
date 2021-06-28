import { storiesOf } from '@storybook/react'
import { action } from 'mobx'
import { reaction } from 'mobx'
import { runInAction } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { observer } from 'mobx-react'
import { now } from 'mobx-utils'
import { Component } from 'react'
import React from 'react'

import { SeededRandom } from '../../../../../shared/base/random/seeded_random'
import { range } from '../../../../../shared/base/range'
import { fullscreen } from '../../../storybook/fullscreen'
import { Classification } from '../../classifications'
import { Lut } from '../../lut'
import { ClassifiedImageModel } from '../model'
import { ClassifiedImageView } from '../view'

import imageUrl from './image.jpg'

storiesOf('classifier.classified_image', module)
  .addDecorator(fullscreen)
  .add('renders statically', () => {
    const random = SeededRandom.of('classifier')
    const lut = generateLut(random)
    const model = ClassifiedImageModel.of({ lut })
    return <ClassifiedImageViewHarness model={model} random={random} />
  })
  .add('renders animated', () => {
    const random = SeededRandom.of('classifier')
    const lut = generateLut(random)
    const model = ClassifiedImageModel.of({ lut })
    return <ClassifiedImageViewHarness model={model} random={random} animate />
  })

@observer
class ClassifiedImageViewHarness extends Component<{
  model: ClassifiedImageModel
  random: SeededRandom
  animate?: boolean
}> {
  async componentDidMount() {
    const image = await loadImage(imageUrl)
    runInAction(() => (this.props.model.rawImage = { type: 'image', image }))
    disposeOnUnmount(
      this,
      reaction(() => this.props.animate && now('frame'), this.update),
    )
  }

  render() {
    return <ClassifiedImageView model={this.props.model} />
  }

  @action.bound
  private update() {
    const percentageFull = 0.4
    range(100).forEach(() => {
      const {
        random,
        model: { lut },
      } = this.props
      const randomIndex = random.integer(0, lut.data.length)
      const randomClassification =
        random.float() <= percentageFull
          ? random.choice(classifications)
          : Classification.Unclassified
      lut.set(randomIndex, randomClassification)
    })
  }
}

function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const image = new Image()
    image.onload = () => resolve(image)
    image.onerror = () => reject()
    image.src = url
  })
}

const classifications = Object.freeze([
  Classification.White,
  Classification.Green,
  Classification.Yellow,
  Classification.Orange,
  Classification.Cyan,
  Classification.Magenta,
])

function generateLut(random: SeededRandom, percentageFull = 0.4) {
  return Lut.generate({ x: 4, y: 4, z: 4 }, () => {
    return random.float() <= percentageFull
      ? random.choice(classifications)
      : Classification.Unclassified
  })
}
