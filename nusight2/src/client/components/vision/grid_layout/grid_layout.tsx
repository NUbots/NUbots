import { Component } from "react";
import React from "react";
import { computed } from "mobx";
import { action } from "mobx";
import { observable } from "mobx";
import { observer } from "mobx-react";
import { ContentRect } from "react-measure";
import Measure from "react-measure";
import { debounce } from "throttle-debounce";

import { objectFit } from "../../three/three";

@observer
export class GridLayout extends Component<{
  itemAspectRatio: number;
  children: React.ReactNode;
}> {
  @observable.ref private accessor width?: number;
  @observable.ref private accessor height?: number;

  render() {
    const gridTemplateColumns = this.gridTemplate;
    return (
      <Measure onResize={this.onContainerResize} bounds={true}>
        {({ measureRef }) => (
          <div ref={measureRef} className="grid w-full h-full" style={{ gridTemplateColumns }}>
            {React.Children.map(this.props.children, (child) => (
              <div style={{ position: "relative" }}>{child}</div>
            ))}
          </div>
        )}
      </Measure>
    );
  }

  private onContainerResize = debounce(
    80,
    action((contentRect: ContentRect) => {
      this.width = contentRect.bounds?.width;
      this.height = contentRect.bounds?.height;
    }),
  );

  @computed
  get gridTemplate() {
    const n = React.Children.count(this.props.children);
    if (!this.width || !this.height || n < 2) {
      return "1fr";
    }
    const cols = bestFitCols({
      container: { width: this.width, height: this.height },
      itemAspectRatio: this.props.itemAspectRatio,
      numItems: n,
    });
    return "1fr ".repeat(cols);
  }
}

export function bestFitCols({
  container,
  itemAspectRatio,
  numItems,
}: {
  container: { width: number; height: number };
  itemAspectRatio: number;
  numItems: number;
}) {
  const arr = [];
  for (let cols = 1; cols <= numItems; cols++) {
    const rows = Math.ceil(numItems / cols);
    const { width, height } = objectFit(
      {
        width: container.width / cols,
        height: container.height / rows,
      },
      {
        type: "contain",
        aspect: itemAspectRatio,
      },
    );
    arr.push({
      cols,
      coverage: (width * height * numItems) / (container.height * container.width),
    });
  }
  arr.sort((a, b) => b.coverage - a.coverage);
  return arr[0].cols;
}
