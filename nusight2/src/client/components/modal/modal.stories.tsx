import React from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Button } from "../button/button";
import { lightAndDarkDecorator } from "../storybook/color_mode";

import { Modal } from "./view";

const meta: Meta<typeof Modal> = {
  title: "components/Modal",
  component: Modal,

  decorators: [lightAndDarkDecorator({ horizontal: true })],
};

export default meta;

type Story = StoryObj<typeof Modal>;

function PlaceholderContent() {
  return (
    <>
      <p className="mt-2">
        Lorem ipsum dolor sit amet consectetur adipisicing elit. Sapiente est molestias nobis fugiat qui illo maiores
        ipsum perspiciatis aperiam deserunt?
      </p>
      <p className="mt-2">
        Lorem ipsum, dolor sit amet consectetur adipisicing elit. Nulla, est fugiat enim culpa quos nam.
      </p>
    </>
  );
}

export const Default: Story = {
  name: "Default",
  render: () => {
    const [show, setShow] = React.useState(false);

    return (
      <>
        <Button onClick={() => setShow(true)}>Open Modal</Button>
        {show ? (
          <Modal onClose={() => setShow(false)}>
            <div className="max-w-xl p-8">
              <h1 className="text-xl font-bold -mt-2 mb-3">Default Modal</h1>
              <a
                className="text-nusight-500 hover:underline focus:underline"
                href="https://google.com"
                target="_blank"
                rel="noopener noreferrer"
              >
                Here is a tabbable link
              </a>
              <PlaceholderContent />
              <br />
              <Button onClick={() => setShow(false)}>Confirm</Button>
            </div>
          </Modal>
        ) : null}
        <br />
        <br />
        <Button>Button After Modal</Button>
      </>
    );
  },
};

export const WithoutTabbableContent: Story = {
  name: "Without Tabbable Content",
  render: () => {
    const [show, setShow] = React.useState(false);

    return (
      <>
        <Button onClick={() => setShow(true)}>Open Modal</Button>
        {show ? (
          <Modal onClose={() => setShow(false)}>
            <div className="max-w-xl p-8">
              <h1 className="text-xl font-bold -mt-2 mb-3">Modal without tabbable content</h1>
              <p>This modal has no keyboard-tabbable content.</p>
              <PlaceholderContent />
            </div>
          </Modal>
        ) : null}
        <br />
        <br />
        <Button>Button After Modal</Button>
      </>
    );
  },
};

export const WithCloseTriggers: Story = {
  name: "Close Triggers",
  render: () => {
    const [showModalA, setShowModalA] = React.useState(false);
    const [showModalB, setShowModalB] = React.useState(false);
    const [showModalC, setShowModalC] = React.useState(false);

    return (
      <>
        <Button onClick={() => setShowModalA(true)}>Modal without default close button</Button>
        {showModalA ? (
          <Modal onClose={() => setShowModalA(false)} closeWith={{ closeButton: false }}>
            <div className="max-w-xl p-8">
              <h1 className="text-xl font-bold -mt-2 mb-3">Modal without default close button</h1>
              <p>This modal has no default close button, use the button below to close.</p>
              <PlaceholderContent />
              <br />
              <Button onClick={() => setShowModalA(false)}>Close Modal</Button>
            </div>
          </Modal>
        ) : null}

        <br />
        <br />

        <Button onClick={() => setShowModalB(true)}>Modal that closes on backdrop click</Button>
        {showModalB ? (
          <Modal onClose={() => setShowModalB(false)} closeWith={{ backdropClick: true }}>
            <div className="max-w-xl p-8">
              <h1 className="text-xl font-bold -mt-2 mb-3">Modal that closes on backdrop click</h1>
              <p>You can close this modal by clicking the backdrop.</p>
              <PlaceholderContent />
            </div>
          </Modal>
        ) : null}

        <br />
        <br />

        <Button onClick={() => setShowModalC(true)}>Modal that doesn’t close on Escape key</Button>
        {showModalC ? (
          <Modal onClose={() => setShowModalC(false)} closeWith={{ escapeKey: false }}>
            <div className="max-w-xl p-8">
              <h1 className="text-xl font-bold -mt-2 mb-3">Modal that doesn’t close on Escape key</h1>
              <p>
                You cannot close this modal by pressing <kbd className="font-bold">Esc</kbd>.
              </p>
              <PlaceholderContent />
            </div>
          </Modal>
        ) : null}
      </>
    );
  },
};
