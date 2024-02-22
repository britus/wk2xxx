## wk2xxx
This repository contains the Linux kernel driver WK2XXX (WK2124, WK2168, WK2212)
SPI to RS232,RS485 switch.

### Device tree nodes (example Firefly AIO-RK3568J IPC industrial tablet)
```
/ {
	spi1: spi@fe620000 {
		compatible = "rockchip,rk3568-spi";
		reg = <0x0 0xfe620000 0x0 0x1000>;
		interrupts = <GIC_SPI 104 IRQ_TYPE_LEVEL_HIGH>;
		#address-cells = <1>;
		#size-cells = <0>;
		clocks = <&cru CLK_SPI1>, <&cru PCLK_SPI1>;
		clock-names = "spiclk", "apb_pclk";
		dmas = <&dmac0 22>, <&dmac0 23>;
		dma-names = "tx", "rx";
		pinctrl-names = "default", "high_speed";
		pinctrl-0 = <&spi1m0_cs0 &spi1m0_cs1 &spi1m0_pins>;
		pinctrl-1 = <&spi1m0_cs0 &spi1m0_cs1 &spi1m0_pins_hs>;
		status = "disabled";
	};
};

&spi1 {
	status = "okay";
	max-freq = <48000000>;
	dev-port = <0>;
	pinctrl-0 = <&spi1m1_pins>;
	pinctrl-1 = <&spi1m1_pins_hs>;

	spi_wk2xxx: spi_wk2xxx@00 {
		status = "okay";
		compatible = "wkmic,wk2124_spi";
		reg = <0x00>;
		spi-max-frequency = <10000000>;
		power-gpio = <&pca9555 PCA_IO1_7 GPIO_ACTIVE_HIGH>;
		reset-gpio = <&pca9555 PCA_IO1_1 GPIO_ACTIVE_HIGH>;
		irq-gpio = <&gpio0 RK_PA6 IRQ_TYPE_EDGE_FALLING>;
		cs-gpio = <&gpio3 RK_PA1 GPIO_ACTIVE_HIGH>;
		/* rk3399 driver support SPI_CPOL | SPI_CPHA | SPI_CS_HIGH */
		//spi-cpha;     /* SPI mode: CPHA=1 */
		//spi-cpol;     /* SPI mode: CPOL=1 */
		//spi-cs-high;
	};
};

```

### Makefile snippet
```
/* SPI to UART switch */
obj-$(CONFIG_SPI_WK2XXX) += spi-wk2xxx.o
ccflags-$(CONFIG_SPI_WK2XXX_DEBUG) += -DDEBUG
```

### Kconfig snippet
```
config SPI_WK2XXX
	tristate "wk2xxx SPI to UART switch"
	depends on SPI && OF
	help
	  This enables the wk2xxx SPI switch to UART chip. You need
	  a device tree node to describe the driver. Important DT
	  properties:
	    * irq-gpio
	  Optional DT properties (see options.)
	    * power-gpio
	    * reset-gpio
	    * cs-gpio

config SPI_WK2XXX_GPIO_RST
	bool "wk2xxx Using reset gpio"
	depends on SPI_WK2XXX
	default y
	help
	  This enables the usage of a reset gpio.
	  DT property 'reset-gpio'

config SPI_WK2XXX_GPIO_CS
	bool "wk2xxx Using chip select gpio"
	depends on SPI_WK2XXX
	default y
	help
	  This enables the usage of a chip select gpio.
	  DT property 'cs-gpio'

config SPI_WK2XXX_GPIO_PWR
	bool "wk2xxx Using power gpio"
	depends on SPI_WK2XXX
	default y
	help
	  This enables the usage of a power gpio.
	  DT property 'power-gpio'

config SPI_WK2XXX_GPIO_RS485
	bool "wk2xxx Function RS485"
	depends on SPI_WK2XXX
	default y
	help
	  This enables the function RS485.

config SPI_WK2XXX_DEBUG
	bool "wk2xxx Debug output"
	depends on SPI_WK2XXX 
	help
	  This enables detailed driver debug output.
```

### Short log output
```
[ 1816.708554]  [0:       modprobe:31698] wk2xxxspi spi1.0: SPI driver for SPI to UART chip WK2XXX
[ 1816.708613]  [0:       modprobe:31698] wk2xxxspi spi1.0: V2.5 on 2024.02.15
[ 1816.795152]  [2:       modprobe:31698] spi1.0: ttysWK0 at I/O 0x1 (irq = 0, base_baud = 691200) is a wk2xxx
[ 1816.795642]  [2:       modprobe:31698] spi1.0: ttysWK1 at I/O 0x2 (irq = 0, base_baud = 691200) is a wk2xxx
[ 1816.795943]  [2:       modprobe:31698] spi1.0: ttysWK2 at I/O 0x3 (irq = 0, base_baud = 691200) is a wk2xxx
[ 1816.796209]  [2:       modprobe:31698] spi1.0: ttysWK3 at I/O 0x4 (irq = 0, base_baud = 691200) is a wk2xxx
[ 1816.796569]  [2:       modprobe:31698] wk2xxxspi spi1.0: Driver successfully installed.
```

### Full debug output loading module
```
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: SPI driver for SPI to UART chip WK2XXX
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: V2.5 on 2024.02.15
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_rstgpio_parse_dt: enter
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_rstgpio_parse_dt: Using reset gpio=209
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_reset: Chip reset complete
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_pwrgpio_parse_dt: start
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_pwrgpio_parse_dt: Using power gpio=215
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_csgpio_parse_dt: enter
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_csgpio_parse_dt: Using CS gpio=97
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_irq_parse_dt: enter
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_spi_irq_parse_dt: Using irq=133 gpio=6
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: Chip reg read GENA=0x30
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: Chip fnc=0xf5 GENA=0x35
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: Chip fnc=0xff GENA=0x3f
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: Initialize serial ports
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_config_port: called. flags=0x1
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_request_port: called.
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_type: called. port type=1
Feb 15 13:16:26 firefly kernel: spi1.0: ttysWK0 at I/O 0x1 (irq = 0, base_baud = 691200) is a wk2xxx
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: uart port=1 registred. ret=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_config_port: called. flags=0x1
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_request_port: called.
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_type: called. port type=1
Feb 15 13:16:26 firefly kernel: spi1.0: ttysWK1 at I/O 0x2 (irq = 0, base_baud = 691200) is a wk2xxx
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: uart port=2 registred. ret=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_config_port: called. flags=0x1
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_request_port: called.
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_type: called. port type=1
Feb 15 13:16:26 firefly kernel: spi1.0: ttysWK2 at I/O 0x3 (irq = 0, base_baud = 691200) is a wk2xxx
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: uart port=3 registred. ret=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_config_port: called. flags=0x1
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_request_port: called.
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_type: called. port type=1
Feb 15 13:16:26 firefly kernel: spi1.0: ttysWK3 at I/O 0x4 (irq = 0, base_baud = 691200) is a wk2xxx
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: wk2xxx_probe: uart port=4 registred. ret=0x0
Feb 15 13:16:26 firefly kernel: wk2xxxspi spi1.0: Driver successfully installed.
```

### Open device with minicom -D /dev/ttysWK3
```
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_startup: port1=1 port2=2 port3=3 port4=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_startup: line1=0 line2=1 line3=2 line4=3
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_startup: port=4 gena=0x38 gier=0x8 sier=0x3 scr=0x3 fcr=0xfc
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_enable_ms: called.
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_startup: exit port:4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0xcbd termios.c_iflag=0x500
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=47 baud1=0 pres=0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=9600
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0x0 baud1=0x47 pres=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x6
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0x1cb2 termios.c_iflag=0x1
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=5 baud1=0 pres=0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=115200
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0x0 baud1=0x5 pres=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0x80001cb2 termios.c_iflag=0x1
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=5 baud1=0 pres=0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=115200
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0x0 baud1=0x5 pres=0x0
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:17:38 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
```

### Type some keys on the keyboard
```
Feb 15 13:18:52 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx_proc: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: fsr=0x0 tfcnt=0x0 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_count=100 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_chars=0x73
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x1
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x0
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx_proc: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: fsr=0x0 tfcnt=0x0 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_count=100 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_chars=0x2e
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x1
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x0
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_start_tx_proc: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: fsr=0x0 tfcnt=0x0 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_count=100 port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: tx_chars=0x64
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: enter
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_irq: exit
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x1
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: gifr=8 gier=8 sier1=0 sier2=0 sier3=0 sier4=7 sifr1=0 sifr2=0 sifr3=0 sifr4=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_port_irq: port=4 sifr=4 sier=7
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: enter port=4
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: port=4 FSR=0x0
Feb 15 13:18:53 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: enter port=4
Feb 15 13:18:54 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_tx: exit port=4
Feb 15 13:18:54 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_chars: exit port=4
Feb 15 13:18:54 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
Feb 15 13:18:54 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: enter
Feb 15 13:18:54 firefly kernel: wk2xxxspi spi1.0: wk2xxx_ist: exit
```

### Closing minicom
```
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0x80000cb0 termios.c_iflag=0x1
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=ff baud1=ff pres=0
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=0
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0xff baud1=0xff pres=0x0
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:19:49 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0x80001cb2 termios.c_iflag=0x1
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=5 baud1=0 pres=0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=115200
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0x0 baud1=0x5 pres=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_get_mctrl: called.
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x6
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: termios.c_cflag=0xcbd termios.c_iflag=0x500
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: baud0=47 baud1=0 pres=0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: freq=11059200 baudrate=9600
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: port=4 lcr=0x0 fwcr=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: baud0=0x0 baud1=0x47 pres=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: conf_wk2xxx_subport: exit port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_termios: exit port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_empty: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_tx_empty: exit port=4 tx_empty=0x1 fsr=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_set_mctrl: called. mctrl=0x0
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_rx: enter port:4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_rx: exit port:4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_rx_proc: enter port:4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_shutdown: enter port=4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_stop_rx_proc: exit port:4
Feb 15 13:19:50 firefly kernel: wk2xxxspi spi1.0: wk2xxx_shutdown: exit port=4
```
