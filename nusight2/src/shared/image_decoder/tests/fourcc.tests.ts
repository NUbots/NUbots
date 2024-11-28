import { describe, expect, it } from "vitest";
import { describe, expect, it } from "vitest";

import { fourcc } from "../fourcc";
import { fourccToString } from "../fourcc";

describe("fourcc", () => {
  it("Calculates fourcc codes from strings", () => {
    expect(fourcc("GREY")).toEqual(0x59455247);
    expect(fourcc("Y12 ")).toEqual(0x20323159);
    expect(fourcc("Y16 ")).toEqual(0x20363159);
    expect(fourcc("GRBG")).toEqual(0x47425247);
    expect(fourcc("RGGB")).toEqual(0x42474752);
    expect(fourcc("GBRG")).toEqual(0x47524247);
    expect(fourcc("BGGR")).toEqual(0x52474742);
    expect(fourcc("GR12")).toEqual(0x32315247);
    expect(fourcc("RG12")).toEqual(0x32314752);
    expect(fourcc("GB12")).toEqual(0x32314247);
    expect(fourcc("BG12")).toEqual(0x32314742);
    expect(fourcc("GR16")).toEqual(0x36315247);
    expect(fourcc("RG16")).toEqual(0x36314752);
    expect(fourcc("GB16")).toEqual(0x36314247);
    expect(fourcc("BG16")).toEqual(0x36314742);
    expect(fourcc("Y411")).toEqual(0x31313459);
    expect(fourcc("UYVY")).toEqual(0x59565955);
    expect(fourcc("YUYV")).toEqual(0x56595559);
    expect(fourcc("YM24")).toEqual(0x34324d59);
    expect(fourcc("RGB3")).toEqual(0x33424752);
    expect(fourcc("JPEG")).toEqual(0x4745504a);
  });
});

describe("fourccToString", () => {
  it("Gives strings for fourcc codes", () => {
    expect(fourccToString(0x59455247)).toEqual("GREY");
    expect(fourccToString(0x20323159)).toEqual("Y12 ");
    expect(fourccToString(0x20363159)).toEqual("Y16 ");
    expect(fourccToString(0x47425247)).toEqual("GRBG");
    expect(fourccToString(0x42474752)).toEqual("RGGB");
    expect(fourccToString(0x47524247)).toEqual("GBRG");
    expect(fourccToString(0x52474742)).toEqual("BGGR");
    expect(fourccToString(0x32315247)).toEqual("GR12");
    expect(fourccToString(0x32314752)).toEqual("RG12");
    expect(fourccToString(0x32314247)).toEqual("GB12");
    expect(fourccToString(0x32314742)).toEqual("BG12");
    expect(fourccToString(0x36315247)).toEqual("GR16");
    expect(fourccToString(0x36314752)).toEqual("RG16");
    expect(fourccToString(0x36314247)).toEqual("GB16");
    expect(fourccToString(0x36314742)).toEqual("BG16");
    expect(fourccToString(0x31313459)).toEqual("Y411");
    expect(fourccToString(0x59565955)).toEqual("UYVY");
    expect(fourccToString(0x56595559)).toEqual("YUYV");
    expect(fourccToString(0x34324d59)).toEqual("YM24");
    expect(fourccToString(0x33424752)).toEqual("RGB3");
    expect(fourccToString(0x4745504a)).toEqual("JPEG");
  });
});
