import React from "react";
import { observer } from "mobx-react";
import * as THREE from "three";
import { Matrix4 } from "three";

@observer
export class GridView extends React.Component<{}> {
  render() {
    return (
      <object3D>
        <primitive object={this.createGrids()} position={[0, 0, 0]}></primitive>
      </object3D>
    );
  }

  private buildHorizontalLine(x1: number, x2: number, y: number, width: number) {
    const length = x2 - x1;
    const hLine = new THREE.PlaneGeometry(length, width);
    hLine.applyMatrix4(new Matrix4().makeTranslation(x1 + length * 0.5, y, 0));
    return hLine;
  }

  private buildVerticalLine(y1: number, y2: number, x: number, width: number) {
    const length = y2 - y1;
    const vLine = new THREE.PlaneGeometry(width, length);
    vLine.applyMatrix4(new Matrix4().makeTranslation(x, y1 + length * 0.5, 0));
    return vLine;
  }

  private createGrids() {
    const size = 20;
    const largeGrid = this.buildGrid(size, 1, new THREE.Color("#E3E3E3"), 0.01);
    // Prevent z-fighting
    largeGrid.translateZ(-0.001);

    const grid = new THREE.Group();
    grid.add(largeGrid);
    grid.rotateX(Math.PI / 2); // Rotate by 90 degrees

    return grid;
  }

  private buildGrid(size: number, step: number, color: any, linewidth: number) {
    const grid = new THREE.Group();

    for (let i = -size / 2; i <= size / 2; i += step) {
      // Create horizontal lines
      const hLine = this.buildHorizontalLine(-size / 2, size / 2, i, linewidth);
      // hLine.translateZ(0.001); // Slight offset to prevent z-fighting
      const hLineMesh = new THREE.Mesh(hLine, new THREE.MeshBasicMaterial({ color }));
      grid.add(hLineMesh);

      // Create vertical lines
      const vLine = this.buildVerticalLine(-size / 2, size / 2, i, linewidth);
      // vLine.translateZ(0.001); // Slight offset
      const vLineMesh = new THREE.Mesh(vLine, new THREE.MeshBasicMaterial({ color }));
      grid.add(vLineMesh);
    }

    grid.rotateX(-Math.PI / 2); // Rotate by 90 degrees

    return grid;
  }
}
