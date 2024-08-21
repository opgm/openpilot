# OPGM openpilot fork
> Latest build info
> - date: {{time.Now.Format "January 02, 2006"}}
> - upstream commit: [`{{.Env.COMMIT | strings.Trunc 10}}`](https://github.com/commaai/openpilot/tree/{{.Env.COMMIT}})

OPGM is a fork of [openpilot](https://github.com/commaai/openpilot/), an open source driver assistance system. Please consult the official documentation for most questions.

## How to use:
1. Ensure your vehicle is on the supported vehicle list (see below).
2. Install a Comma 3 or Comma 3X and wiring harness
   - (Optional) If you have an EV, a [Comma pedal](https://shop.tlbb.ca/) is highly recommended.
3. Type `opgm.cc` in the software installation screen
4. (Optional) Turn on openpilot longitudinal control in the settings for "[redneck ACC](https://www.youtube.com/watch?v=41wZ1EAmf94)"

## Supported vehicles list
If your car works on stock openpilot, you should not use OPGM. The software will likely work without issue, but support from OPGM is not guaranteed since you have official support upstream.

### Non-ACC GM cars
These cars **must** come with factory LKAS or there is no guarantee that openpilot will work!
* 2016-2019 Chevrolet Volt
* 2017-2019 Chevrolet Bolt EV
* 2020-2023 Chevrolet Bolt EV/EUV
* Chevrolet Equinox/GMC Terrain
* Chevrolet Tahoe/GMC Yukon
* Chevrolet Suburban
* Chevrolet Trailblazer
* Chevrolet Malibu
* Cadillac XT5
* Cadillac CT6
#### Required hardware
Buy the GM harness bundle with your C3X.

If you want to use a Comma Pedal, you will need the following instead:
- C3X
- GM harness **wires only**
- [old style harness box](https://oneclone.net/product/fresh-orange-juice/)

### SDGM cars
These cars have a Serial Data Gateway Module and require a special wiring harness.
* 2019 Chevrolet Volt
* Cadillac XT4
* Chevrolet Traverse
* Buick Baby Enclave
#### Required hardware
TODO


> At this time, the Global B (VIP) architecture is unsupported due to CAN bus encryption. If you have a vehicle on one of these architectures and consider yourself to be a hacker/tinkerer type, we would love to hear from you!

### Fingerprinting your vehicle
Note! You likely do not have to do this. Try installing OPGM first, and only proceed with the fingerprinting process if
it does not work out of the box.

You will first need to set up [SSH access](https://github.com/commaai/openpilot/wiki/SSH) to your device. Then, follow
the fingerprinting instructions posted [here](https://github.com/commaai/openpilot/wiki/Fingerprinting#fingerprinting-10).

> You must follow the **v1.0** instructions. Fingerprint v2.0 is not yet supported for GM vehicles.

Reach out in the [OPGM discord channel](#discussion) with questions, or after you have fingerprinted your vehicle so it
can be added to OPGM.

## Installation
### Hardware
OPGM supports the Comma Three development platform; legacy support for Comma Two is not guaranteed. Some older development
branches may work on Comma Two. Use at your own risk.

Verify first that your vehicle has LKAS. Verify as well that it has a forward-facing camera; you can do this by removing
the plastic cover on the windshield behind the rearview mirror. If you see a silver rectangular camera, you're good to go.

If you have vehicle supported by upstream openpilot, buy the corresponding hardware from Comma.

If you have a vehicle without ACC, buy the [Bolt EV/EUV kit from Comma](https://comma.ai/shop/comma-three). If you have
a Bolt EV/EUV, it is **strongly** recommended to purchase a pedal interceptor for the best experience. (You may always
add one later, if you want to try OPGM before committing.)

You may buy a pedal interceptor from the following vendors:
* [TinyBear](https://www.etsy.com/listing/952895642/openpilot-comma-pedal-non-customizable?variation0=3013902165)

### Software
For the latest stable build, use the install URL: `opgm.cc`

> #### Installation troubleshooting
> If the installation fails partway through, try moving your device closer to your WiFi router.
>   * You may plug the Comma Three into a USB power source delivering at least 2A of current.

Ensure that your car is *completely* powered off during software installation, otherwise you may get a "no panda" error.
To be sure that your car is completely powered down:
1. Turn the car on and off
2. Open and close the driver's door
3. Wait 5 minutes

## Known issues
* Curve hugging and laneline crossing are known issues of upstream openpilot.
* Officially supported cars (e.g. ACC Volt and Bolt) may have issues fingerprinting on OPGM. Currently, the only workaround is to hardcode your fingerprint in the `/data/openpilot/launch_env.sh` file.

## Discussion
Come join us on the OPGM channel in the [openpilot community discord](https://discord.gg/KGWEdwSnCU)!

## Contributing
Contributions are welcome, but please note the following:
1. All pull requests should be opened against the `dev` branch.
2. OPGM does not accept requests to support cars that meet Comma's minimum supportability requirements. Work on supporting such cars should be done upstream on stock openpilot.
3. OPGM generally does not accept features outside the very narrow scope of "minimum functionality". Such features should be proposed upstream or included in other community forks.

## Credits
### Current contributors
* [comma.ai](https://comma.ai) for openpilot
* nworby (BDFL)
* mochi86420

### Past contributors
* jshuler (father of OPGM)
* twilsonco
* Many others

# License
OPGM is under the MIT license. See [LICENSE](LICENSE) for more information.
