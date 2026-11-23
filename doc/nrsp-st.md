# nRSP-ST Internals

Inside the nRSP-ST is a standard Raspberry Pi CM4. The SDR device is connected over USB (`1df7:3040`).

The following GPIOs are known:
| GPIO | Idle | Function | Notes |
|---|---|---|---|
| 0 | high | ATSHA204A data, single wire mode | ID_SD. Wake reply `04 11 33 43`, chip is blank and unlocked. |
| 1 | high | ATSHA204A pin 6 | ID_SC. Unused in single wire mode. |
| 2 | high | Flash connector VBUS detect, input | Pulled low when VBUS is detected on the FLASH connector. Can be driven low by the Pi to route its internal USB to that connector. |
| 7 | high | External VBUS detect, input | Pulled low when VBUS is detected on the USB-B connector.  |
| 8 | high | USB switch side, output | Drive low to expose the SDR to the USB-B connector, drive high or let float to access the SDR via the CM4.  |
| 9 | high | Unknown | Has a pull-up on the board, function is not known. |
| 10 | high | SDR section power enable, output | Drive low to turn off the power to the SDR section, drive high or let float to turn it on. |
| 14 | high | UART TXD | Expansion connector pin 4: GPIO14 |
| 15 | high | UART RXD | Expansion connector pin 3: GPIO15 |
| 16 |  | Expansion connector pin 5, UART CTS | Through an unfitted resistor: GPIO16 |
| 17 |  | Expansion connector pin 2, UART RTS | Through an unfitted resistor: GPIO17 |
| 26 | high | WL_nDisable | Driving low disables the WiFi radio. |
| 27 | high | BT_nDisable | Driving low disables the Bluetooth radio. |

## Connectors

| Connector | Wiring |
|---|---|
| Console | 1 [GND], 2 [GPIO17 RTS (unfitted)], 3 [GPIO15 RXD], 4 [GPIO14 TXD], 5 [GPIO16 CTS (unfitted)].


