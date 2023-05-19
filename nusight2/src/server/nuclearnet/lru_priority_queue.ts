type Item<K, V> = { key: K; values: V[] };

/**
 * A priority queue which favours values whose keys are the least recently used (LRU).
 *
 * Optionally can limit the queue capacity per key, favouring the newest entries and dropping the oldest ones.
 */
export class LruPriorityQueue<K, V> {
  private size: number = 0;
  private readonly capacityPerKey?: number;
  private readonly queue: Item<K, V>[] = [];
  private readonly map: Map<K, Item<K, V>> = new Map();

  constructor({ capacityPerKey }: { capacityPerKey?: number } = {}) {
    this.capacityPerKey = capacityPerKey;
  }

  add(key: K, value: V) {
    let item = this.map.get(key);
    if (!item) {
      item = { key, values: [] };
      this.map.set(key, item);
      this.queue.push(item);
    }
    if (this.capacityPerKey && item.values.length >= this.capacityPerKey) {
      item.values.shift();
      this.size--;
    }
    item.values.push(value);
    this.size++;
  }

  pop(): V | undefined {
    if (!this.size) {
      return;
    }
    /* tslint:disable-next-line prefer-for-of */
    for (let i = 0; i < this.queue.length; i++) {
      const item = this.queue.shift()!;
      this.queue.push(item);
      const value = item.values.shift();
      if (value) {
        this.size--;
        return value;
      }
    }
  }
}
