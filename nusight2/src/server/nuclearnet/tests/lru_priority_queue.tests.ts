import { LruPriorityQueue } from "../lru_priority_queue";

describe("LruPriorityQueue", () => {
  let queue: LruPriorityQueue<string, string>;

  beforeEach(() => {
    queue = new LruPriorityQueue<string, string>();
  });

  /** Pop and return all items in the queue, in priority order. */
  function popAll<K, V>(queue: LruPriorityQueue<K, V>): V[] {
    const items: V[] = [];
    let item: V | undefined;
    /* tslint:disable-next-line no-conditional-assignment */
    while ((item = queue.pop())) {
      items.push(item);
    }
    return items;
  }

  it("returns undefined when no keys have been added", () => {
    expect(queue.pop()).toBe(undefined);
  });

  it("returns values currently in queue and then undefined", () => {
    queue.add("foo", "foo_1");
    expect(queue.pop()).toBe("foo_1");
    expect(queue.pop()).toBe(undefined);
    queue.add("bar", "bar_1");
    expect(queue.pop()).toBe("bar_1");
    expect(queue.pop()).toBe(undefined);
  });

  it("orders values by least recently used key", () => {
    queue.add("foo", "foo_1");
    queue.add("foo", "foo_2");
    queue.add("bar", "bar_1");
    queue.add("bar", "bar_2");
    queue.add("baz", "baz_1");
    queue.add("baz", "baz_2");
    expect(popAll(queue)).toEqual(["foo_1", "bar_1", "baz_1", "foo_2", "bar_2", "baz_2"]);
  });

  it("drops older values when keys are over capacity", () => {
    const queue = new LruPriorityQueue<string, string>({ capacityPerKey: 2 });
    queue.add("foo", "foo_1");
    queue.add("foo", "foo_2");
    queue.add("foo", "foo_3");
    queue.add("foo", "foo_4");
    queue.add("bar", "bar_1");
    queue.add("bar", "bar_2");
    queue.add("bar", "bar_3");
    expect(popAll(queue)).toEqual(["foo_3", "bar_2", "foo_4", "bar_3"]);
  });
});
