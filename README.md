# AT86RF215 IEEE 802.15.4 driver (out-of-tree)

Stand-alone build of the Linux driver for the Microchip AT86RF215 dual-band
IEEE 802.15.4 transceiver. Only legacy O-QPSK (250 kb/s) is supported:

- 2.4 GHz radio: channel page 0, channels 11-26
- sub-GHz radio: channel page 2, channels 1-10

Requires a recent kernel (6.13 or newer) with `CONFIG_MAC802154` and
`CONFIG_SPI` enabled.

## Build

    make                      # against the running kernel
    make KDIR=/path/to/build  # against another kernel build tree
    sudo make install

## Device tree

See `atmel,at86rf215.yaml` for the binding. Example:

    &spi0 {
        ieee802154@0 {
            compatible = "atmel,at86rf215";
            reg = <0>;
            spi-max-frequency = <25000000>;
            interrupt-parent = <&gpio>;
            interrupts = <23 IRQ_TYPE_LEVEL_HIGH>;
            reset-gpios = <&gpio 24 GPIO_ACTIVE_LOW>;
        };
    };

Both radios are registered as separate wpan PHYs.
