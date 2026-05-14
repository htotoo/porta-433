# Porta-433

## Overview

Porta-433 is a stripped-down custom firmware based on Mayhem for the PortaPack platform. It is focused on RTL433 support and keeps the firmware intentionally minimal by removing most of the standard Mayhem applications.

The goal of this project is to provide a PortaPack firmware build that:

- Is based on Mayhem
- Can be flashed via the web using the same general workflow as Mayhem firmware
- Uses a custom firmware image for Porta-433
- Includes RTL433 protocol support
- Parses and decodes supported RTL433 signals

## Release Status

🚧 **Alpha Release**

This project is an early alpha release. It is incomplete, actively changing, and should be considered experimental.

## Features

- Mayhem-based firmware foundation
- Web flashing support in the same style as Mayhem
- Custom Porta-433 firmware image
- RTL433 protocol support
- RTL433 signal parsing and decoding
- Reduced application set for a lightweight build

## Known limits

The Portapack is limited in memory and computing power. RTL433 is designed to be a computer app, so the PortaPack version may drop valid samples or just detect the nth repetion. 
The screen is also small for that many information.


## Flashing

Porta-433 can be flashed through the web using the same general process as Mayhem firmware, but a custom firmware build must be selected for this project. https://hackrf.app

## Warnings

Use at own risk.

No liability is accepted for any damage, loss, malfunction, hardware issue, data loss, or other consequences resulting from the use of this firmware.

## Stability

As an alpha release, Porta-433 may change frequently and may contain bugs or incomplete features. Behavior, protocol support, and available functionality may change without notice.

## Revert to Mayhem

Just go to the https://hackrf.app and flash the latest Stable or Nightly

## Contributions

PRs are welcome.

If your change is firmware-related but not RTL433-specific, please submit it to the Mayhem firmware project instead.

## Thanks

This project is made possible thanks to:

- The **Mayhem** project and its contributors
- The **RTL433** project and its contributors
- The **HackRF** team members and community

Thank you all for making this possible.


