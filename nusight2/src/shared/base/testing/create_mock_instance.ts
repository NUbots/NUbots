import Mocked = jest.Mocked;

export function createMockInstance<T>(ctor: new (...args: any[]) => T): Mocked<T> {
  return stubMethods<T>(Object.create(ctor.prototype));
}

function stubMethods<T>(obj: Mocked<T>, mock: Mocked<T> = obj, stubbed: Set<string> = new Set()): Mocked<T> {
  for (const prop of Object.getOwnPropertyNames(obj)) {
    if (!stubbed.has(prop)) {
      const descriptor = Object.getOwnPropertyDescriptor(obj, prop);
      if (obj !== Object.prototype && prop !== "constructor" && descriptor && typeof descriptor.value === "function") {
        Object.defineProperty(mock, prop, { ...descriptor, value: jest.fn() });
      }
      stubbed.add(prop);
    }
  }

  const proto = Object.getPrototypeOf(obj);

  if (proto) {
    return stubMethods<T>(proto, mock, stubbed);
  }

  return mock;
}
