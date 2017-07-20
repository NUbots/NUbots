import { Clock } from './clock'

type Task = {
  id: number,
  nextTime: number,
  period?: number,
  fn(): void,
}

export class FakeClock implements Clock {
  private nextId: number
  private time: number
  private tasks: Task[]

  constructor() {
    this.nextId = 0
    this.time = 0
    this.tasks = []
  }

  public static of() {
    return new FakeClock()
  }

  public now(): number {
    return this.time
  }

  public performanceNow(): number {
    return this.time
  }

  public setTimeout(fn: () => void, seconds: number): () => void {
    const id = this.nextId++
    this.addTask({ id, nextTime: this.now() + seconds, fn })
    return () => this.removeTask(id)
  }

  public setInterval(fn: () => void, seconds: number): () => void {
    const id = this.nextId++
    this.addTask({ id, nextTime: this.now() + seconds, period: seconds, fn })
    return () => this.removeTask(id)
  }

  public setImmediate(fn: () => void): () => void {
    const id = this.nextId++
    this.addTask({ id, nextTime: this.now() + Number.MIN_VALUE, fn })
    return () => this.removeTask(id)
  }

  public tick(delta: number = 1) {
    const newTime = this.now() + delta

    while (this.tasks.length > 0 && this.tasks[0].nextTime <= newTime) {
      const task = this.tasks[0]
      this.consumeTask(task)
    }

    this.time = newTime
  }

  public runAllTimers() {
    const limit = 1000
    let i = 0

    while (this.tasks.length > 0) {
      if (i > limit) {
        throw new Error(`Exceeded clock task limit of ${limit}, possibly caused by a infinite task loop?`)
      }
      const task = this.tasks[0]
      this.consumeTask(task)
      i++
    }
  }

  public runOnlyPendingTimers() {
    const limit = 1000
    let i = 0

    const lastTastTime = this.tasks.length > 0 && this.tasks[this.tasks.length - 1].nextTime

    while (this.tasks.length > 0 && this.tasks[0].nextTime <= lastTastTime) {
      if (i > limit) {
        throw new Error(`Exceeded clock task limit of ${limit}, possibly caused by a infinite task loop?`)
      }
      const task = this.tasks[0]
      this.consumeTask(task)
      i++
    }
  }

  public runTimersToTime(time: number) {
    const limit = 1000
    let i = 0

    while (this.tasks.length > 0 && this.tasks[0].nextTime <= time) {
      if (i > limit) {
        throw new Error(`Exceeded clock task limit of ${limit}, possibly caused by a infinite task loop?`)
      }
      const task = this.tasks[0]
      this.consumeTask(task)
      i++
    }
  }

  private addTask(task: Task) {
    this.tasks.push(task)
    this.sortTasks()
  }

  private sortTasks() {
    this.tasks.sort((t1, t2) => {
      return t1.nextTime - t2.nextTime
    })
  }

  private consumeTask(task: Task) {
    this.time = task.nextTime
    if (task.period != null) {
      task.nextTime += task.period
      this.sortTasks()
    } else {
      this.tasks.shift()
    }
    task.fn()
  }

  private removeTask(taskId: number) {
    for (let i = 0; i < this.tasks.length; i++) {
      if (this.tasks[i].id == taskId) {
        this.tasks.splice(i, 1)
        break
      }
    }
  }
}
