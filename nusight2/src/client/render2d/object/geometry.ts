import { ArcGeometry } from "../geometry/arc_geometry";
import { ArrowGeometry } from "../geometry/arrow_geometry";
import { CircleGeometry } from "../geometry/circle_geometry";
import { LineGeometry } from "../geometry/line_geometry";
import { PathGeometry } from "../geometry/path_geometry";
import { PolygonGeometry } from "../geometry/polygon_geometry";
import { TextGeometry } from "../geometry/text_geometry";

export type Geometry =
  | ArcGeometry
  | ArrowGeometry
  | CircleGeometry
  | LineGeometry
  | PathGeometry
  | PolygonGeometry
  | TextGeometry;
