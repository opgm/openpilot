# OPGM: openpilot for Unsupported GM Cars

OPGM is a fork of [openpilot](https://github.com/commaai/openpilot/), the open source driver assistance system by comma.ai. This fork enables support for GM vehicles that are not officially supported upstream, while staying as close as possible to stock openpilot behavior.

> OPGM strives to be **"stock openpilot for unsupported GM cars."**

To achieve this, OPGM follows these development guidelines:

1. **Non-GM code is never modified** by OPGM. Shared/common code is modified only when absolutely necessary.
2. **Upstream-supported GM vehicles** are only patched to enable additional functionality (e.g., stop-and-go support, lower min engagement speed).
3. **Control schemes already supported by upstream openpilot** must not be modified. Alternate control schemes, such as `pedal long` or `CC long`, are exempt from this rule.

For general openpilot documentation, setup, and developer guidance, refer to the [official openpilot repo](https://github.com/commaai/openpilot/).

---

## Features

- Support for GM Global A vehicles with LKAS but no ACC
- `pedal long`: Longitudinal control using a pedal interceptor
  - Enables full regenerative braking on Bolt EV/EUV (even those with ACC)
- `CC long`: Non-ACC cruise control adjustment, aka [“redneck ACC”](https://www.youtube.com/watch?v=41wZ1EAmf94)
  - ⚠️ Cannot reduce speed below stock cruise control limits (usually 24mph)

---

## Supported Vehicles

Support is focused on GM Global A vehicles with factory LKAS. For the latest list, see [CARS.md](https://github.com/opgm/opendbc/blob/master/docs/CARS.md)

If your vehicle isn’t listed, but has LKAS and a front-facing camera, it might still be supported or supportable. See the “contributing” section below.

---

## Installation

Hardware installation is the same as for any vehicle with the standard GM harness in upstream openpilot.

For software installation, type `opgm/release` into the custom software field, or `opgm/nightly` if you want to live on the bleeding edge.

> **Note:** For EVs without ACC, a [pedal interceptor](https://www.etsy.com/listing/952895642/openpilot-comma-pedal-non-customizable?variation0=3013902165) is **strongly recommended** for a better experience.

---

## Contributing

All development occurs on the `master` branch. Contributions are welcome: feel free to open a PR.

Bug reports and vehicle porting discussions happen in the OPGM channel on the [openpilot community Discord](https://discord.gg/KGWEdwSnCU).

---

## Known Issues

- Curve hugging and lane line drift are inherited from upstream openpilot.
- Upstream-supported GM cars (e.g. ACC Volt/Bolt) may fail to fingerprint automatically in OPGM.

---

## License

Licensed under the MIT License. See [LICENSE](LICENSE) for details.
