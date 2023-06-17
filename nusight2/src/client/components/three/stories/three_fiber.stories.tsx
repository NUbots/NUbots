import { observable } from "mobx";
import { observer } from "mobx-react";
import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { reaction } from "mobx";
import { now } from "mobx-utils";
import { useEffect } from "react";

import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { fullscreen } from "../../storybook/fullscreen";
import { PerspectiveCamera, ThreeFiber } from "../three_fiber";

type Story = StoryObj<typeof BoxVisualiser>;

const meta: Meta<typeof BoxVisualiser> = {
  title: "components/ThreeFiber",
  parameters: {
    layout: "fullscreen",
  },
  decorators: [fullscreen],
};

export default meta;

export const StaticScene: Story = {
  name: "static scene",
  render: () => <BoxVisualiser />,
};

export const AnimatedScene: Story = {
  name: "animated scene",
  render: () => <BoxVisualiser animate />,
};

type Model = { boxes: BoxModel[] };
type BoxModel = { id: string; color: string; size: number; position: Vector3; rotation: Vector3 };

const BoxVisualiser = (props: { animate?: boolean }) => {
  // TODO: Use `useLocalObservable` when mobx is updated.
  const model = observable({
    boxes: [
      { id: "1", color: "red", size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { id: "2", color: "green", size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { id: "3", color: "blue", size: 1, position: Vector3.of(), rotation: Vector3.of() },
    ],
  });
  useEffect(() => {
    function update(now: number) {
      const t = (2 * Math.PI * now) / (20 * 1000);
      const n = model.boxes.length;
      for (const [i, box] of model.boxes.entries()) {
        const position = Vector2.fromPolar(1, (i * 2 * Math.PI) / n + t);
        box.position = new Vector3(position.x, position.y, 0);
        box.rotation = new Vector3(Math.cos(3 * t + i), Math.cos(5 * t + i), Math.cos(7 * t + i));
      }
    }

    update(0);
    if (props.animate) {
      return reaction(() => now("frame"), update);
    }
  }, []);
  return (
    <ThreeFiber>
      <Scene model={model} />
    </ThreeFiber>
  );
};

const Scene = observer(({ model }: { model: Model }) => (
  <>
    <PerspectiveCamera args={[60, 1, 0.5, 10]} position={[0, 0, 4]} />
    <pointLight position={[0, 0, 4]} />
    {model.boxes.map((box) => (
      <BoxView key={box.id} box={box} />
    ))}
  </>
));

const BoxView = observer(({ box }: { box: BoxModel }) => (
  <mesh
    position={box.position.toArray()}
    rotation={box.rotation.toArray()}
    scale={Vector3.fromScalar(box.size).toArray()}
  >
    <boxGeometry args={[1, 1, 1]} />
    <meshPhongMaterial color={box.color} />
  </mesh>
));
