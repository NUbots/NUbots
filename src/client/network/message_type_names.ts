import { message } from '../../shared/proto/messages'
import { MessageType } from './global_network'
import { Message } from './global_network'

export class MessageTypePath {
  private cache: Map<any, string>
  private searchObject: any

  public constructor() {
    this.searchObject = { message }
    this.cache = new Map()
  }

  public static of = (() => {
    let instance: MessageTypePath
    return (): MessageTypePath => {
      if (!instance) {
        instance = new MessageTypePath()
      }
      return instance
    }
  })()

  public getPath(messageType: MessageType<Message>): string {
    if (!this.cache.has(messageType)) {
      const path = findPath(this.searchObject, value => value === messageType)
      if (!path) {
        throw new Error(`Unknown type given ${messageType}`)
      }
      this.cache.set(messageType, path)
    }
    return String(this.cache.get(messageType))
  }
}

/**
 * Recursively searches an object for a given value, if found it will return the object path to that value.
 *
 * e.g. findPath({ a: { b: { c: 'd' } } }, v => v === 'd') // 'a.b.c'
 */
function findPath(obj: any, isSubject: (value: any) => boolean, path: string[] = []): string|undefined {
  if (isSubject(obj)) {
    return path.join('.')
  }
  if (typeof obj !== 'object') {
    return undefined
  }
  if (Array.isArray(obj)) {
    throw new Error('Array support not implemented')
  }
  for (const key of Object.getOwnPropertyNames(obj)) {
    const foundPath = findPath(obj[key], isSubject, [...path, key])
    if (foundPath) {
      return foundPath
    }
  }
  return undefined
}
