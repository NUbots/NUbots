import bounds from 'binary-search-bounds'
import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { now } from 'mobx-utils'

import { Transform } from '../../../math/transform'
import { Vector2 } from '../../../math/vector2'
import { BasicAppearance } from '../../../render2d/appearance/basic_appearance'
import { LineAppearance } from '../../../render2d/appearance/line_appearance'
import { LineGeometry } from '../../../render2d/geometry/line_geometry'
import { PathGeometry } from '../../../render2d/geometry/path_geometry'
import { TextGeometry } from '../../../render2d/geometry/text_geometry'
import { Geometry } from '../../../render2d/object/geometry'
import { Group } from '../../../render2d/object/group'
import { Shape } from '../../../render2d/object/shape'
import { CheckedState } from '../../checkbox_tree/model'
import { DataSeries } from '../model'
import { TreeData } from '../model'

import { LineChartModel } from './model'

export class LineChartViewModel {
  constructor(private model: LineChartModel) {}

  static of = createTransformer((model: LineChartModel): LineChartViewModel => {
    return new LineChartViewModel(model)
  })

  @computed
  get bufferSeconds() {
    return this.model.bufferSeconds
  }

  @computed
  get camera(): Transform {
    const yScale = 0.9 / (this.maxValue - this.minValue) // 0.9 so there is a little extra above and below the plot
    const xScale = 1 / this.model.bufferSeconds

    return Transform.of({
      scale: {
        x: xScale,
        y: yScale,
      },
    })
  }

  @computed
  get dataSeries(): DataSeries[] {
    const series: DataSeries[] = []
    const queue: (TreeData | DataSeries)[] = [this.model.treeData]

    while (queue.length > 0) {
      const elem = queue.pop()!

      if (elem instanceof DataSeries && elem.checked === CheckedState.Checked) {
        series.push(elem)
      } else if (!(elem instanceof DataSeries)) {
        queue.push(...elem.values())
      }
    }

    return series
  }

  @computed
  get scene(): Group {
    return Group.of({
      children: [this.chart, this.axis],
    })
  }

  @computed
  get axis(): Group {
    return Group.of({
      children: [this.yAxis, this.xAxis],
    })
  }

  @computed
  get yAxis(): Group {
    // Work out the distance between our major and minor grid lines
    const nMinor = 4
    const range = this.maxValue - this.minValue
    const digits = Math.floor(Math.log10(range * 0.5))
    const major = Math.pow(10, digits)
    const minor = major / nMinor
    const offset = this.minValue + (this.maxValue - this.minValue) / 2

    const lines: Shape<Geometry>[] = []

    // Make our major and minor lines
    let lineNo = 0
    for (
      let y = Math.floor(this.minValue / major) * major - major;
      y <= this.maxValue + major;
      y += minor
    ) {
      const geometry = LineGeometry.of({
        origin: Vector2.of(-this.model.bufferSeconds / 2, y - offset),
        target: Vector2.of(this.model.bufferSeconds / 2, y - offset),
      })

      if (lineNo % nMinor === 0) {
        // Major gridline
        lines.push(
          Shape.of(
            geometry,
            LineAppearance.of({
              stroke: {
                color: '#555555',
                width: 1,
                nonScaling: true,
              },
            }),
          ),
        )

        lines.push(
          Shape.of(
            TextGeometry.of({
              text: y.toPrecision(2).toString(),
              worldScale: true,
              textAlign: 'end',
              fontSize: '1em',
              x: this.model.bufferSeconds / 2,
              y: y - offset,
            }),
            BasicAppearance.of({
              fill: {
                color: '#000000',
              },
            }),
          ),
        )
      } else {
        // Minor gridline
        lines.push(
          Shape.of(
            geometry,
            LineAppearance.of({
              stroke: {
                color: '#999999',
                width: 0.5,
                nonScaling: true,
              },
            }),
          ),
        )
      }

      lineNo++
    }

    return Group.of({
      children: lines,
    })
  }

  @computed
  get xAxis(): Group {
    // Work out our min/max value
    const max = this.now
    const min = max - this.model.bufferSeconds
    const yRange = this.maxValue - this.minValue

    // Work out the distance between our major and minor grid lines
    const nMinor = 4
    const range = this.model.bufferSeconds
    const digits = Math.floor(Math.log10(range))
    const major = Math.pow(10, digits)
    const minor = major / nMinor
    const offset = min + (max - min) / 2

    const lines: Shape<Geometry>[] = []

    // Make our major and minor lines
    let lineNo = 0
    for (let x = Math.floor(min / major) * major - major; x <= max + major; x += minor) {
      const geometry = LineGeometry.of({
        origin: Vector2.of(x - offset, -yRange),
        target: Vector2.of(x - offset, yRange),
      })

      if (lineNo % nMinor === 0) {
        // Major gridline
        lines.push(
          Shape.of(
            geometry,
            LineAppearance.of({
              stroke: {
                color: '#555555',
                width: 1,
                nonScaling: true,
              },
            }),
          ),
        )
      } else {
        // Minor gridline
        lines.push(
          Shape.of(
            geometry,
            LineAppearance.of({
              stroke: {
                color: '#999999',
                width: 0.5,
                nonScaling: true,
              },
            }),
          ),
        )
      }

      lineNo++
    }

    return Group.of({
      children: lines,
    })
  }

  @computed
  get chart() {
    // Get our min and max values
    const minValue = this.model.yMin === 'auto' ? this.minValue : this.model.yMin
    const maxValue = this.model.yMax === 'auto' ? this.maxValue : this.model.yMax

    return Group.of({
      transform: Transform.of({
        translate: {
          x: -(this.now - this.model.bufferSeconds / 2),
          y: -(minValue + (maxValue - minValue) / 2),
        },
      }),
      children: this.dataSeries.map(series => this.makeLines(series)),
    })
  }

  @computed
  get maxValue(): number {
    if (this.model.yMax !== 'auto') {
      return this.model.yMax
    } else if (this.dataSeries.length === 0) {
      return 1
    } else {
      const max = this.dataSeries.reduce((maxValue, series: DataSeries) => {
        // Get the range we are viewing
        let end = this.now + series.timeDelta
        let start = end - this.model.bufferSeconds

        const values = series.series
        end = Math.max(
          0,
          bounds.lt(values, Vector2.of(), p => p.x - end),
        )
        start = Math.max(
          0,
          bounds.lt(values, Vector2.of(), p => p.x - start),
        )

        return values.slice(start, end).reduce((max, value) => {
          return Math.max(max, value.y)
        }, maxValue)
      }, -Number.MAX_VALUE)
      return max === -Number.MAX_VALUE ? 1 : max
    }
  }

  @computed
  get minValue(): number {
    if (this.model.yMin !== 'auto') {
      return this.model.yMin
    } else if (this.dataSeries.length === 0) {
      return -1
    } else {
      const min = this.dataSeries.reduce((minValue, series: DataSeries) => {
        // Get the range we are viewing
        let end = this.now + series.timeDelta
        let start = end - this.model.bufferSeconds

        const values = series.series
        end = Math.max(
          0,
          bounds.lt(values, Vector2.of(), p => p.x - end),
        )
        start = Math.max(
          0,
          bounds.lt(values, Vector2.of(), p => p.x - start),
        )

        return values.slice(start, end).reduce((min, value) => {
          return Math.min(min, value.y)
        }, minValue)
      }, Number.MAX_VALUE)
      return min === Number.MAX_VALUE ? -1 : min
    }
  }

  @computed
  get now() {
    return now('frame') / 1000 - this.model.startTime
  }

  private makeLines(series: DataSeries): Group {
    // Get the range we are viewing
    let end = this.now + series.timeDelta
    let start = end - this.model.bufferSeconds

    let values = series.series
    end = Math.max(
      0,
      bounds.lt(values, Vector2.of(), p => p.x - end),
    )
    start = Math.max(
      0,
      bounds.lt(values, Vector2.of(), p => p.x - start),
    )
    values = values.slice(start, end)

    // If we have no values, don't draw the line
    if (values.length === 0) {
      return Group.of()
    }

    const lines = []

    if (series.highlight) {
      lines.push(
        Shape.of(
          PathGeometry.of(values),
          LineAppearance.of({
            stroke: {
              color: '#ffff00',
              width: 8,
              nonScaling: true,
            },
          }),
        ),
      )
    }

    lines.push(
      Shape.of(
        PathGeometry.of(values),
        LineAppearance.of({
          stroke: {
            color: series.color,
            width: 2,
            nonScaling: true,
          },
        }),
      ),
    )

    // Apply our time delta
    return Group.of({
      transform: Transform.of({
        translate: Vector2.of(-series.timeDelta, 0),
      }),
      children: lines,
    })
  }
}
