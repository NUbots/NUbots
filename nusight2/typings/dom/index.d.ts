interface Element {
  requestPointerLock(): void
}

interface Document {
  pointerLockElement: Element
}
