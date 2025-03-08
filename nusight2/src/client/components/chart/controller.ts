import { action } from "mobx";

import { BrowserSystemClock } from "../../../client/time/browser_clock";
import { Clock } from "../../../shared/time/clock";
import { CheckedState } from "../checkbox_tree/model";
import { TreeNodeModel } from "../checkbox_tree/model";

import { ChartModel } from "./model";
import { TreeViewModel } from "./view_model";

export class ChartController {
  private colorPool: string[] = [
    "#f0a3ff",
    "#0075dc",
    "#993f00",
    "#4c005c",
    "#191919",
    "#005c31",
    "#2bce48",
    "#ffcc99",
    "#808080",
    "#94ffb5",
    "#8f7c00",
    "#9dcc00",
    "#c20088",
    "#003380",
    "#ffa405",
    "#ffa8bb",
    "#426600",
    "#ff0010",
    "#5ef1f2",
    "#00998f",
    "#e0ff66",
    "#740aff",
    "#990000",
    "#ffff80",
    "#ffff00",
    "#ff5005",
  ];
  private usedColors: Set<string> = new Set(["#ffffff"]);

  constructor(
    private model: ChartModel,
    private clock: Clock,
  ) {
    this.model = model;
    this.clock = clock;
  }

  static of(opts: { model: ChartModel }) {
    return new ChartController(opts.model, BrowserSystemClock);
  }

  @action
  onColorChange = (color: string, node: TreeNodeModel): void => {
    if (node instanceof TreeViewModel) {
      node.color = color;
    } else {
      throw new Error(`Unsupported node: ${node}`);
    }
  };

  @action
  private setHighlight(node: TreeNodeModel, set: boolean) {
    if (node.leaf) {
      const leaf = node as TreeViewModel;
      leaf.highlight = set;
    } else {
      node.children.forEach((n) => this.setHighlight(n, set));
    }
  }

  onHighlight = (node: TreeNodeModel): void => {
    this.setHighlight(node, true);
  };

  onUnhighlight = (node: TreeNodeModel): void => {
    this.setHighlight(node, false);
  };

  @action
  onNodeExpand = (node: TreeNodeModel): void => {
    node.expanded = !node.expanded;
  };

  private assignColor(node: TreeNodeModel) {
    if (node.leaf) {
      const leaf = node as TreeViewModel;

      // Try to maintain colors if possible
      if (!this.usedColors.has(leaf.color)) {
        this.usedColors.add(leaf.color);
      } else {
        const freeColor = this.colorPool.some((c) => {
          if (!this.usedColors.has(c)) {
            leaf.color = c;
            this.usedColors.add(c);
            return true;
          }
          return false;
        });

        // There were no colors left :(
        if (!freeColor) {
          // Make a random color that's not close to white (keep the sum of parts lower than 255)
          let r = Math.random();
          let g = Math.random();
          let b = Math.random();

          // If we are over 2/3 close to white, normalise us back
          if (r + g + b >= 2) {
            r *= 2 / 3;
            g *= 2 / 3;
            b *= 2 / 3;
          }

          r = Math.floor(r * 255);
          g = Math.floor(g * 255);
          b = Math.floor(b * 255);

          const rS = ("0" + (+r).toString(16)).slice(-2);
          const gS = ("0" + (+g).toString(16)).slice(-2);
          const bS = ("0" + (+b).toString(16)).slice(-2);

          leaf.color = `#${rS}${gS}${bS}`;
        }
      }
    } else {
      node.children.forEach((n) => this.assignColor(n));
    }
  }

  private clearColor(node: TreeNodeModel) {
    if (node.leaf) {
      const leaf = node as TreeViewModel;
      this.usedColors.delete(leaf.color);
    } else {
      node.children.forEach((n) => this.clearColor(n));
    }
  }

  @action
  onNodeCheck = (node: TreeNodeModel) => {
    if (node.checked === CheckedState.Checked) {
      node.checked = CheckedState.Unchecked;
      this.clearColor(node);
    } else if (node.checked === CheckedState.Unchecked) {
      node.checked = CheckedState.Checked;
      this.assignColor(node);
    } else {
      if (this.model.tree.usePessimisticToggle) {
        node.checked = CheckedState.Unchecked;
        this.clearColor(node);
      } else {
        node.checked = CheckedState.Checked;
        this.assignColor(node);
      }
    }
  };
}
