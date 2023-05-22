declare module "binary" {
  type Vars = { [key: string]: any };

  interface Binary {
    vars: any;
    buffer(key: string, size: number): Binary;

    word8le(key: string): Binary;
    word8lu(key: string): Binary;
    word8ls(key: string): Binary;
    word8be(key: string): Binary;
    word8bu(key: string): Binary;
    word8bs(key: string): Binary;

    word16le(key: string): Binary;
    word16lu(key: string): Binary;
    word16ls(key: string): Binary;
    word16be(key: string): Binary;
    word16bu(key: string): Binary;
    word16bs(key: string): Binary;

    word32le(key: string): Binary;
    word32lu(key: string): Binary;
    word32ls(key: string): Binary;
    word32be(key: string): Binary;
    word32bu(key: string): Binary;
    word32bs(key: string): Binary;

    word64le(key: string): Binary;
    word64lu(key: string): Binary;
    word64ls(key: string): Binary;
    word64be(key: string): Binary;
    word64bu(key: string): Binary;
    word64bs(key: string): Binary;

    into(key: string, cb: (this: Binary, vars: Vars) => void): Binary;
    tap(cb: (this: Binary, vars: Vars) => void): Binary;
    scan(key: string, buffer: Buffer): Binary;
    loop(cb: (end: () => void, vars: Vars) => void): Binary;
    flush(): void;
  }

  interface BinaryStatic {
    parse(buffer: Buffer): Binary;
  }

  const value: BinaryStatic;
  export = value;
}
