/*
 * (C) Copyright 2022 WKIC Ltd. by Xu XunWei Tech, Xuxunwei
 * (C) Copyright 2024 EoF Software Labs, B. Eschrich
 * SPDX-License-Identifier:	GPL-2.0+
 */

/* ---------------------------------------------------
 *   FILE NAME: wk2xxx_spi.c   
 *   DEMO Version: 2.4 Data: 2022-07-24
 *   DESCRIPTION: Implements an interface for the wk2xxx of spi interface
 *   WKIC Ltd. By  Xu XunWei Tech  
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "linux/version.h"
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <uapi/linux/sched.h>
#include <uapi/linux/sched/types.h>

#define DRIVER_DESC "SPI driver for SPI to UART chip WK2XXX"
#define VERSION_DESC "V2.5 on 2024.02.15"
#define DRIVER_AUTHOR "Xuxunwei/B.Eschrich"

/*************The debug control **********************************/
#if defined(CONFIG_SPI_WK2XXX_DEBUG)
#define _DEBUG_WK_FUNCTION
#define _DEBUG_WK_RX
#define _DEBUG_WK_TX
#define _DEBUG_WK_IRQ
#define _DEBUG_WK_VALUE
#define _DEBUG_WK_TEST
#endif

/*************Functional control interface************************/
#define WK_WORK_KTHREAD
#define WK_FIFO_FUNCTION
//#define  WK_FLOWCTRL_FUNCTION

#if defined(CONFIG_SPI_WK2XXX_GPIO_RS485)
#define WK_RS485_FUNCTION
#endif

#if defined(CONFIG_SPI_WK2XXX_GPIO_RST)
#define WK_RSTGPIO_FUNCTION
#endif

#if defined(CONFIG_SPI_WK2XXX_GPIO_CS)
#define WK_CSGPIO_FUNCTION
#endif

#if defined(CONFIG_SPI_WK2XXX_GPIO_PWR)
#define WK_PWRGPIO_FUNCTION
#endif

/*************SPI control interface******************************/
#define SPI_LEN_LIMIT 30 //MAX<=255

/*************Uart Setting interface******************************/
#define WK2XXX_TXFIFO_LEVEL (0x01) /* TX FIFO level */
#define WK2XXX_RXFIFO_LEVEL (0x40) /* RX FIFO level */

#define WK2XXX_STATUS_PE 1
#define WK2XXX_STATUS_FE 2
#define WK2XXX_STATUS_BRK 4
#define WK2XXX_STATUS_OE 8

static DEFINE_MUTEX(wk2xxxs_lock);
static DEFINE_MUTEX(wk2xxxs_reg_lock);
static DEFINE_MUTEX(wk2xxxs_global_lock);

/******************************************/
#define WK2_NR_PORTS 4
//
#define SERIAL_WK2XXX_MAJOR 207
#define CALLOUT_WK2XXX_MAJOR 208
#define MINOR_START 5
//wk2xxx hardware configuration
#define WK_SPI_SPEED 10000000
//#define WK_CRASTAL_CLK (24000000)
#define WK_CRASTAL_CLK (11059200)
#define WK2_ISR_PASS_LIMIT 2
#define PORT_WK2XXX 1
/******************************************/

/************** WK2XXX register definitions********************/
/*wk2xxx  Global register address defines*/
#define WK2XXX_GENA_REG 0X00 /*Slave UART Clock Set */
#define WK2XXX_GRST_REG 0X01 /*Reset Slave UART*/
#define WK2XXX_GMUT_REG 0X02 /*Master UART Control*/
#define WK2XXX_GIER_REG 0X10 /*Slave UART Interrupt Enable */
#define WK2XXX_GIFR_REG 0X11 /*Slave UART Interrupt Flag*/
#define WK2XXX_GPDIR_REG 0X21 /*GPIO Direction*/ /*WK2168/WK2212*/
#define WK2XXX_GPDAT_REG 0X31 
/* ^^ GPIO Data Input and Data Output*/ /*WK2168/WK2212*/

/*****************************
****wk2xxx  slave uarts  register address defines****
******************************/
#define WK2XXX_SPAGE_REG 0X03 /*Slave UART Register page selection*/
#define WK2XXX_PAGE1 1
#define WK2XXX_PAGE0 0

/*PAGE0**/
#define WK2XXX_SCR_REG 0X04 /*Slave UART Transmitter and Receiver Enable*/
#define WK2XXX_LCR_REG 0X05 /* Line Control */
#define WK2XXX_FCR_REG 0X06 /* FIFO control */
#define WK2XXX_SIER_REG 0X07 /* Interrupt enable */
#define WK2XXX_SIFR_REG 0X08 /* Interrupt Identification */
#define WK2XXX_TFCNT_REG 0X09 /* TX FIFO counter */
#define WK2XXX_RFCNT_REG 0X0A /* RX FIFO counter */
#define WK2XXX_FSR_REG 0X0B /* FIFO Status */
#define WK2XXX_LSR_REG 0X0C /* Line Status */
#define WK2XXX_FDAT_REG	0X0D 
/*  ^^ Write transmit FIFO data or Read receive FIFO data */
#define WK2XXX_FWCR_REG 0X0E /* Flow  Control */
#define WK2XXX_RS485_REG 0X0F /* RS485 Control */

/*PAGE1*/
#define WK2XXX_BAUD1_REG 0X04 /* Divisor Latch High */
#define WK2XXX_BAUD0_REG 0X05 /* Divisor Latch Low */
#define WK2XXX_PRES_REG 0X06 /* Divisor Latch Fractional Part */
#define WK2XXX_RFTL_REG 0X07 /* Receive FIFO Trigger Level */
#define WK2XXX_TFTL_REG 0X08 /* Transmit FIFO Trigger Level */
#define WK2XXX_FWTH_REG 0X09 /*Flow control trigger high level*/
#define WK2XXX_FWTL_REG 0X0A /*Flow control trigger low level*/
#define WK2XXX_XON1_REG 0X0B /* Xon1 word */
#define WK2XXX_XOFF1_REG 0X0C /* Xoff1 word */
#define WK2XXX_SADR_REG 0X0D /*RS485 auto address*/
#define WK2XXX_SAEN_REG 0X0E /*RS485 auto address mask*/
#define WK2XXX_RRSDLY_REG 0X0F /*RTS delay when transmit in RS485*/

//wkxxx register bit defines
/*GENA register*/
#define WK2XXX_GENA_UT4EN_BIT 0x08
#define WK2XXX_GENA_UT3EN_BIT 0x04
#define WK2XXX_GENA_UT2EN_BIT 0x02
#define WK2XXX_GENA_UT1EN_BIT 0x01
/*GRST register*/
#define WK2XXX_GRST_UT4SLEEP_BIT 0x80
#define WK2XXX_GRST_UT3SLEEP_BIT 0x40
#define WK2XXX_GRST_UT2SLEEP_BIT 0x20
#define WK2XXX_GRST_UT1SLEEP_BIT 0x10
#define WK2XXX_GRST_UT4RST_BIT 0x08
#define WK2XXX_GRST_UT3RST_BIT 0x04
#define WK2XXX_GRST_UT2RST_BIT 0x02
#define WK2XXX_GRST_UT1RST_BIT 0x01
/*GIER register bits*/
#define WK2XXX_GIER_UT4IE_BIT 0x08
#define WK2XXX_GIER_UT3IE_BIT 0x04
#define WK2XXX_GIER_UT2IE_BIT 0x02
#define WK2XXX_GIER_UT1IE_BIT 0x01
/*GIFR register bits*/
#define WK2XXX_GIFR_UT4INT_BIT 0x08
#define WK2XXX_GIFR_UT3INT_BIT 0x04
#define WK2XXX_GIFR_UT2INT_BIT 0x02
#define WK2XXX_GIFR_UT1INT_BIT 0x01
/*SPAGE register bits*/
#define WK2XXX_SPAGE_PAGE_BIT 0x01
/*SCR register bits*/
#define WK2XXX_SCR_SLEEPEN_BIT 0x04
#define WK2XXX_SCR_TXEN_BIT 0x02
#define WK2XXX_SCR_RXEN_BIT 0x01
/*LCR register bits*/
#define WK2XXX_LCR_BREAK_BIT 0x20
#define WK2XXX_LCR_IREN_BIT 0x10

#define WK2XXX_LCR_PAEN_BIT 0x08
#define WK2XXX_LCR_PAM1_BIT 0x04
#define WK2XXX_LCR_PAM0_BIT 0x02
/*ODD Parity*/
#define WK2XXX_LCR_ODD_PARITY 0x0a
/*Even Parity*/
#define WK2XXX_LCR_EVEN_PARITY 0x0c
/*Parity :=0*/
#define WK2XXX_LCR_SPACE_PARITY 0x08
/*Parity :=1*/
#define WK2XXX_LCR_MARK_PARITY 0x0e

#define WK2XXX_LCR_STPL_BIT 0x01

/*FCR register bits*/
#define WK2XXX_FCR_TFEN_BIT 0x08
#define WK2XXX_FCR_RFEN_BIT 0x02
#define WK2XXX_FCR_RFRST_BIT 0x01
/*SIER register bits*/
#define WK2XXX_SIER_FERR_IEN_BIT 0x80
#define WK2XXX_SIER_CTS_IEN_BIT 0x40
#define WK2XXX_SIER_RTS_IEN_BIT 0x20
#define WK2XXX_SIER_XOFF_IEN_BIT 0x10
#define WK2XXX_SIER_TFEMPTY_IEN_BIT 0x08
#define WK2XXX_SIER_TFTRIG_IEN_BIT 0x04
#define WK2XXX_SIER_RXOUT_IEN_BIT 0x02
#define WK2XXX_SIER_RFTRIG_IEN_BIT 0x01
/*SIFR register bits*/
#define WK2XXX_SIFR_FERR_INT_BIT 0x80
#define WK2XXX_SIFR_CTS_INT_BIT 0x40
#define WK2XXX_SIFR_RTS_INT_BIT 0x20
#define WK2XXX_SIFR_XOFF_INT_BIT 0x10
#define WK2XXX_SIFR_TFEMPTY_INT_BIT 0x08
#define WK2XXX_SIFR_TFTRIG_INT_BIT 0x04
#define WK2XXX_SIFR_RXOVT_INT_BIT 0x02
#define WK2XXX_SIFR_RFTRIG_INT_BIT 0x01
/*FSR register bits*/
#define WK2XXX_FSR_RFOE_BIT 0x80
#define WK2XXX_FSR_RFBI_BIT 0x40
#define WK2XXX_FSR_RFFE_BIT 0x20
#define WK2XXX_FSR_RFPE_BIT 0x10

#define WK2XXX_FSR_ERR_MASK 0xF0

#define WK2XXX_FSR_RDAT_BIT 0x08
#define WK2XXX_FSR_TDAT_BIT 0x04
#define WK2XXX_FSR_TFULL_BIT 0x02
#define WK2XXX_FSR_TBUSY_BIT 0x01
/*LSR register bits*/
#define WK2XXX_LSR_BRK_ERROR_MASK 0X0F /* BI, FE, PE, OE bits */
#define WK2XXX_LSR_OE_BIT 0x08
#define WK2XXX_LSR_BI_BIT 0x04
#define WK2XXX_LSR_FE_BIT 0x02
#define WK2XXX_LSR_PE_BIT 0x01
/*FWCR register bits*/
#define WK2XXX_FWCR_RTS_BIT 0x02
#define WK2XXX_FWCR_CTS _BIT 0x01
/*RS485 register bits*/
#define WK2XXX_RS485_RSRS485_BIT 0x40
#define WK2XXX_RS485_ATADD_BIT 0x20
#define WK2XXX_RS485_DATEN_BIT 0x10
#define WK2XXX_RS485_RTSEN_BIT 0x02
#define WK2XXX_RS485_RTSINV_BIT 0x01

#define WK2XXX_GPIO_NAME_POWER "power-gpio"
#define WK2XXX_GPIO_NAME_RESET "reset-gpio"
#define WK2XXX_GPIO_NAME_IRQ "irq-gpio"
#define WK2XXX_GPIO_NAME_CS "cs-gpio"

struct wk2xxx_devtype {
	char name[10];
	int nr_uart;
};

struct wk2xxx_one {
	struct uart_port port; //[WK2_NR_PORTS];
	struct kthread_work start_tx_work;
	struct kthread_work stop_rx_work;
	uint8_t line;
	uint8_t new_lcr_reg;
	uint8_t new_fwcr_reg;
	uint8_t new_scr_reg;
	/*baud register*/
	uint8_t new_baud1_reg;
	uint8_t new_baud0_reg;
	uint8_t new_pres_reg;
};
#define to_wk2xxx_one(p, e) ((container_of((p), struct wk2xxx_one, e)))

struct wk2xxx_port {
	const struct wk2xxx_devtype *devtype;
	struct uart_driver uart;
	struct spi_device *spi_wk;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	unsigned char buf[256];
	struct kthread_worker kworker;
	struct task_struct *kworker_task;
	struct kthread_work irq_work;
	int init_done; /* 1 if probe successfully */

#ifdef WK_CSGPIO_FUNCTION
	int cs_gpio_num;
#endif
#ifdef WK_PWRGPIO_FUNCTION
	int pwr_gpio_num;
#endif
#ifdef WK_RSTGPIO_FUNCTION
	int rst_gpio_num;
#endif
	int irq_num;
	int irq_gpio_num;
	int minor; /* minor number */
	int tx_empty;
	struct wk2xxx_one p[WK2_NR_PORTS];
};

static const struct wk2xxx_devtype wk2124_devtype = {
	.name = "WK2124",
	.nr_uart = 4,
};
static const struct wk2xxx_devtype wk2132_devtype = {
	.name = "WK2132",
	.nr_uart = 2,
};
static const struct wk2xxx_devtype wk2204_devtype = {
	.name = "WK2204",
	.nr_uart = 4,
};
static const struct wk2xxx_devtype wk2168_devtype = {
	.name = "WK2168",
	.nr_uart = 4,
};
static const struct wk2xxx_devtype wk2202_devtype = {
	.name = "WK2202",
	.nr_uart = 2,
};

/* This function reads global chip register */
static int wk2xxx_read_global_reg(struct spi_device *spi, uint8_t reg,
				  uint8_t *dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	struct spi_message msg;
	uint8_t buf_wdat[2];
	uint8_t buf_rdat[2];
	struct spi_transfer index_xfer = {
		.len = 2,
		.speed_hz = WK_SPI_SPEED,
	};
	int status = 0;

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	buf_wdat[0] = 0x40 | reg;
	buf_wdat[1] = 0x00;
	buf_rdat[0] = 0x00;
	buf_rdat[1] = 0x00;
	index_xfer.tx_buf = buf_wdat;
	index_xfer.rx_buf = (void *)buf_rdat;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	if (status) {
		return status;
	}

	*dat = buf_rdat[1];
	return 0;
}

/* This function writes the global register */
static int wk2xxx_write_global_reg(struct spi_device *spi, uint8_t reg,
				   uint8_t dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	struct spi_message msg;
	uint8_t buf_reg[2];
	struct spi_transfer index_xfer = {
		.len = 2,
		.speed_hz = WK_SPI_SPEED,
	};
	int status = 0;

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	/* register index */
	buf_reg[0] = 0x00 | reg;
	buf_reg[1] = dat;
	index_xfer.tx_buf = buf_reg;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	return status;
}

/* This function reads the slave register */
static int wk2xxx_read_slave_reg(struct spi_device *spi, uint8_t port,
				 uint8_t reg, uint8_t *dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	struct spi_message msg;
	uint8_t buf_wdat[2];
	uint8_t buf_rdat[2];
	struct spi_transfer index_xfer = {
		.len = 2,
		.speed_hz = WK_SPI_SPEED,
	};
	int status = 0;

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	buf_wdat[0] = 0x40 | (((port - 1) << 4) | reg);
	buf_wdat[1] = 0x00;
	buf_rdat[0] = 0x00;
	buf_rdat[1] = 0x00;
	index_xfer.tx_buf = buf_wdat;
	index_xfer.rx_buf = (void *)buf_rdat;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	if (status) {
		return status;
	}

	*dat = buf_rdat[1];
	return 0;
}

/* This function writes the slave register */
static int wk2xxx_write_slave_reg(struct spi_device *spi, uint8_t port,
				  uint8_t reg, uint8_t dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	struct spi_message msg;
	uint8_t buf_reg[2];
	struct spi_transfer index_xfer = {
		.len = 2,
		.speed_hz = WK_SPI_SPEED,
	};
	int status = 0;

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	/* register index */
	buf_reg[0] = ((port - 1) << 4) | reg;
	buf_reg[1] = dat;
	index_xfer.tx_buf = buf_reg;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	return status;
}

#define MAX_RFCOUNT_SIZE 256

/* This function reads the fifo register */
static int wk2xxx_read_fifo(struct spi_device *spi, uint8_t port,
			    uint8_t fifolen, uint8_t *dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	uint8_t recive_fifo_data[MAX_RFCOUNT_SIZE + 1] = { 0 };
	uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE + 1] = { 0 };
	struct spi_transfer index_xfer = {
		.len = fifolen + 1,
		.speed_hz = WK_SPI_SPEED,
	};
	struct spi_message msg;
	int status, i;

	if (!(fifolen > 0)) {
		dev_err(&spi->dev, "%s: fifolen error.\n", __func__);
		return -EIO;
	}

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	/* register index */
	transmit_fifo_data[0] = ((port - 1) << 4) | 0xc0;
	index_xfer.tx_buf = transmit_fifo_data;
	index_xfer.rx_buf = (void *)recive_fifo_data;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);
	for (i = 0; i < fifolen; i++)
		*(dat + i) = recive_fifo_data[i + 1];

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	return status;
}

/* This function writes the fifo register */
static int wk2xxx_write_fifo(struct spi_device *spi, uint8_t port,
			     uint8_t fifolen, uint8_t *dat)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	uint8_t recive_fifo_data[MAX_RFCOUNT_SIZE + 1] = { 0 };
	uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE + 1] = { 0 };
	struct spi_transfer index_xfer = {
		.len = fifolen + 1,
		.speed_hz = WK_SPI_SPEED,
	};
	struct spi_message msg;
	int status, i;

	if (!(fifolen > 0)) {
		dev_err(&spi->dev, "%s: fifolen error. fifolen=%d\n", __func__,
			fifolen);
		return -EIO;
	}

	mutex_lock(&wk2xxxs_reg_lock);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 0);
#endif

	spi_message_init(&msg);
	/* register index */
	transmit_fifo_data[0] = ((port - 1) << 4) | 0x80;
	for (i = 0; i < fifolen; i++) {
		transmit_fifo_data[i + 1] = *(dat + i);
	}
	index_xfer.tx_buf = transmit_fifo_data;
	index_xfer.rx_buf = (void *)recive_fifo_data;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);

#ifdef WK_CSGPIO_FUNCTION
	gpio_set_value(priv->cs_gpio_num, 1);
#endif

	mutex_unlock(&wk2xxxs_reg_lock);
	return status;
}

static void conf_wk2xxx_subport(struct uart_port *port);
static void wk2xxx_stop_tx(struct uart_port *port);
static u_int wk2xxx_tx_empty(struct uart_port *port);

static void wk2xxx_rx_chars(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	uint8_t fsr, rx_dat[256] = { 0 };
	uint8_t rfcnt = 0, rfcnt2 = 0;
	unsigned int flg, status = 0, rx_count = 0;
	int rx_num = 0, rxlen = 0;
	int len_rfcnt, len_limit, len_p = 0;
	len_limit = SPI_LEN_LIMIT;

	dev_dbg(&priv->spi_wk->dev, "%s: enter port=%ld\n", __func__,
		one->port.iobase);

	wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase, WK2XXX_FSR_REG,
			      &fsr);
	if (fsr & WK2XXX_FSR_RDAT_BIT) {
		wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase,
				      WK2XXX_RFCNT_REG, &rfcnt);
		if (rfcnt == 0) {
			wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase,
					      WK2XXX_RFCNT_REG, &rfcnt);
		}
		wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase,
				      WK2XXX_RFCNT_REG, &rfcnt2);
		if (rfcnt2 == 0) {
			wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase,
					      WK2XXX_RFCNT_REG, &rfcnt2);
		}
		rfcnt = (rfcnt2 >= rfcnt) ? rfcnt : rfcnt2;
		rxlen = (rfcnt == 0) ? 256 : rfcnt;
	}

#ifdef _DEBUG_WK_RX
	dev_dbg(&priv->spi_wk->dev, "%s: port=%lx fsr=0x%x rxlen=%d\n",
		__func__, one->port.iobase, fsr, rxlen);
#endif

	flg = TTY_NORMAL;

#ifdef WK_FIFO_FUNCTION
	len_rfcnt = rxlen;
	while (len_rfcnt) {
		if (len_rfcnt > len_limit) {
			wk2xxx_read_fifo(priv->spi_wk, one->port.iobase,
					 len_limit, rx_dat + len_p);
			len_rfcnt = len_rfcnt - len_limit;
			len_p = len_p + len_limit;
		} else {
			wk2xxx_read_fifo(priv->spi_wk, one->port.iobase,
					 len_rfcnt, rx_dat + len_p);
			len_rfcnt = 0;
		}
	}
#else
	for (rx_num = 0; rx_num < rxlen; rx_num++) {
		wk2xxx_read_slave_reg(priv->spi_wk, one->port.iobase,
				      WK2XXX_FDAT_REG, &rx_dat[rx_num]);
	}
#endif

	one->port.icount.rx += rxlen;
	for (rx_num = 0; rx_num < rxlen; rx_num++) {
		if (fsr & WK2XXX_FSR_ERR_MASK) {
			fsr &= WK2XXX_FSR_ERR_MASK;
			if (fsr & (WK2XXX_FSR_RFOE_BIT | WK2XXX_FSR_RFBI_BIT |
				   WK2XXX_FSR_RFFE_BIT | WK2XXX_FSR_RFPE_BIT)) {
				if (fsr & WK2XXX_FSR_RFPE_BIT) {
					one->port.icount.parity++;
					status |= WK2XXX_STATUS_PE;
					flg = TTY_PARITY;
				}
				if (fsr & WK2XXX_FSR_RFFE_BIT) {
					one->port.icount.frame++;
					status |= WK2XXX_STATUS_FE;
					flg = TTY_FRAME;
				}
				if (fsr & WK2XXX_FSR_RFOE_BIT) {
					one->port.icount.overrun++;
					status |= WK2XXX_STATUS_OE;
					flg = TTY_OVERRUN;
				}
				if (fsr & WK2XXX_FSR_RFBI_BIT) {
					one->port.icount.brk++;
					status |= WK2XXX_STATUS_BRK;
					flg = TTY_BREAK;
				}
			}
		}

		if (uart_handle_sysrq_char(port, rx_dat[rx_num]))
			continue;

#ifdef _DEBUG_WK_RX
		dev_dbg(&priv->spi_wk->dev, "%s: rx_chars=0x%x\n", __func__,
			rx_dat[rx_num]);
#endif

		uart_insert_char(port, status, WK2XXX_STATUS_OE, rx_dat[rx_num],
				 flg);
		rx_count++;
	}

	if (rx_count > 0) {
#ifdef _DEBUG_WK_RX
		dev_dbg(&priv->spi_wk->dev,
			"%s: push buffer tty flip port=%lx count=%d\n",
			__func__, one->port.iobase, rx_count);
#endif
		tty_flip_buffer_push(&port->state->port);
		rx_count = 0;
	}

	dev_dbg(&priv->spi_wk->dev, "%s: exit port=%ld\n", __func__,
		one->port.iobase);
}

static void wk2xxx_tx_chars(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	uint8_t fsr, tfcnt, dat[1], txbuf[256] = { 0 };
	int count, tx_count, i;
	int len_tfcnt, len_limit, len_p = 0;
	len_limit = SPI_LEN_LIMIT;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	if (one->port.x_char) {
#ifdef _DEBUG_WK_TX
		dev_dbg(&spi->dev, "%s: one->port.x_char=%x port=%ld\n",
			__func__, one->port.x_char, one->port.iobase);
#endif

		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FDAT_REG,
				       one->port.x_char);
		one->port.icount.tx++;
		one->port.x_char = 0;
		goto out;
	}

	if (uart_circ_empty(&one->port.state->xmit) ||
	    uart_tx_stopped(&one->port)) {
		goto out;
	}

	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FSR_REG, &fsr);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_TFCNT_REG, &tfcnt);

#ifdef _DEBUG_WK_TX
	dev_dbg(&spi->dev, "%s: fsr=0x%x tfcnt=0x%x port=%ld\n", __func__, fsr,
		tfcnt, one->port.iobase);
#endif

	if (tfcnt == 0) {
		tx_count = (fsr & WK2XXX_FSR_TFULL_BIT) ? 0 : 256;
#ifdef _DEBUG_WK_TX
		dev_dbg(&spi->dev, "%s: tx_count=%x port=%ld\n", __func__,
			tx_count, one->port.iobase);
#endif
	} else {
		tx_count = 256 - tfcnt;
#ifdef _DEBUG_WK_TX
		dev_dbg(&spi->dev, "%s: tx_count=%x port=%ld\n", __func__,
			tx_count, one->port.iobase);
#endif
	}

	if (tx_count > 200) {
		tx_count = 200;
	}

	i = 0;
	count = tx_count;
	while (count) {
		if (uart_circ_empty(&one->port.state->xmit))
			break;
		txbuf[i] =
			one->port.state->xmit.buf[one->port.state->xmit.tail];
		one->port.state->xmit.tail =
			(one->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
		one->port.icount.tx++;
		i++;
		count = count - 1;
#ifdef _DEBUG_WK_TX
		dev_dbg(&spi->dev, "%s: tx_chars=0x%x\n", __func__,
			txbuf[i - 1]);
#endif
	};

#ifdef WK_FIFO_FUNCTION
	len_tfcnt = i;
	while (len_tfcnt) {
		if (len_tfcnt > len_limit) {
			wk2xxx_write_fifo(spi, one->port.iobase, len_limit,
					  txbuf + len_p);
			len_p = len_p + len_limit;
			len_tfcnt = len_tfcnt - len_limit;
		} else {
			wk2xxx_write_fifo(spi, one->port.iobase, len_tfcnt,
					  txbuf + len_p);
			len_p = len_p + len_tfcnt;
			len_tfcnt = 0;
		}
	}
#else
	for (count = 0; count < i; count++) {
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FDAT_REG,
				       txbuf[count]);
	}
#endif

out:
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FSR_REG, dat);
	fsr = dat[0];

#ifdef _DEBUG_WK_VALUE
	dev_dbg(&spi->dev, "%s: port=%ld FSR=0x%X\n", __func__,
		one->port.iobase, fsr);
#endif

	if (((fsr & WK2XXX_FSR_TDAT_BIT) == 0) &&
	    ((fsr & WK2XXX_FSR_TBUSY_BIT) == 0)) {
		if (uart_circ_chars_pending(&one->port.state->xmit) <
		    WAKEUP_CHARS) {
			uart_write_wakeup(&one->port);
		}
		if (uart_circ_empty(&one->port.state->xmit)) {
			wk2xxx_stop_tx(&one->port);
		}
	}

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

/* port IRQ handler */
static void wk2xxx_port_irq(struct wk2xxx_port *priv, int portno)
{
	struct spi_device *spi = priv->spi_wk;
	struct wk2xxx_one *one = &priv->p[portno];
	unsigned int pass_counter = 0;
	uint8_t sifr, sier;
#ifdef _DEBUG_WK_IRQ
	uint8_t gier, sifr0, sifr1, sifr2, sifr3, sier1, sier0, sier2, sier3,
		gifr;

#endif

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

#ifdef _DEBUG_WK_IRQ
	wk2xxx_read_global_reg(spi, WK2XXX_GIFR_REG, &gifr);
	wk2xxx_read_global_reg(spi, WK2XXX_GIER_REG, &gier);
	wk2xxx_read_slave_reg(spi, 1, WK2XXX_SIFR_REG, &sifr0);
	wk2xxx_read_slave_reg(spi, 2, WK2XXX_SIFR_REG, &sifr1);
	wk2xxx_read_slave_reg(spi, 3, WK2XXX_SIFR_REG, &sifr2);
	wk2xxx_read_slave_reg(spi, 4, WK2XXX_SIFR_REG, &sifr3);
	wk2xxx_read_slave_reg(spi, 1, WK2XXX_SIER_REG, &sier0);
	wk2xxx_read_slave_reg(spi, 2, WK2XXX_SIER_REG, &sier1);
	wk2xxx_read_slave_reg(spi, 3, WK2XXX_SIER_REG, &sier2);
	wk2xxx_read_slave_reg(spi, 4, WK2XXX_SIER_REG, &sier3);
	dev_dbg(&spi->dev,
		"%s: gifr=%x gier=%x sier1=%x sier2=%x sier3=%x sier4=%x "
		"sifr1=%x sifr2=%x sifr3=%x sifr4=%x\n",
		__func__, gifr, gier, sier0, sier1, sier2, sier3, sifr0, sifr1,
		sifr2, sifr3);
#endif

	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIFR_REG, &sifr);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, &sier);

#ifdef _DEBUG_WK_IRQ
	dev_dbg(&spi->dev, "%s: port=%ld sifr=%x sier=%x\n", __func__,
		one->port.iobase, sifr, sier);
#endif

	do {
		if ((sifr & WK2XXX_SIFR_RFTRIG_INT_BIT) ||
		    (sifr & WK2XXX_SIFR_RXOVT_INT_BIT)) {
			wk2xxx_rx_chars(&one->port);
		}

		if ((sifr & WK2XXX_SIFR_TFTRIG_INT_BIT) &&
		    (sier & WK2XXX_SIER_TFTRIG_IEN_BIT)) {
			wk2xxx_tx_chars(&one->port);
			return;
		}

		if (pass_counter++ > WK2_ISR_PASS_LIMIT)
			break;

		wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIFR_REG,
				      &sifr);
		wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG,
				      &sier);

#ifdef _DEBUG_WK_VALUE
		dev_dbg(&spi->dev, "%s: RX/TX sifr=%x sier=%x port=%ld\n",
			__func__, sifr, sier, one->port.iobase);
#endif

	} while ((sifr &
		  (WK2XXX_SIFR_RXOVT_INT_BIT | WK2XXX_SIFR_RFTRIG_INT_BIT)) ||
		 ((sifr & WK2XXX_SIFR_TFTRIG_INT_BIT) &&
		  (sier & WK2XXX_SIER_TFTRIG_IEN_BIT)));

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static void wk2xxx_ist(struct kthread_work *ws)
{
	struct wk2xxx_port *priv =
		container_of(ws, struct wk2xxx_port, irq_work);
	struct spi_device *spi = priv->spi_wk;
	uint8_t gifr, i;

	dev_dbg(&spi->dev, "%s: enter\n", __func__);

	wk2xxx_read_global_reg(spi, WK2XXX_GIFR_REG, &gifr);

	while (1) {
		for (i = 0; i < priv->devtype->nr_uart; ++i) {
			if (gifr & (0x01 << i)) {
				wk2xxx_port_irq(priv, i);
			}
		}

		wk2xxx_read_global_reg(spi, WK2XXX_GIFR_REG, &gifr);
		if (!(gifr & 0x0f)) {
			break;
		}
	}

	dev_dbg(&spi->dev, "%s: exit\n", __func__);
}

/* gpio IRQ handler */
static irqreturn_t wk2xxx_irq(int irq, void *dev_id) //
{
	struct wk2xxx_port *priv = (struct wk2xxx_port *)dev_id;
	struct spi_device *spi = priv->spi_wk;
	bool ret;

	dev_dbg(&spi->dev, "%s: enter\n", __func__);

#ifdef WK_WORK_KTHREAD
	ret = kthread_queue_work(&priv->kworker, &priv->irq_work);
#else
	ret = queue_kthread_work(&priv->kworker, &priv->irq_work);
#endif

	dev_dbg(&spi->dev, "%s: exit\n", __func__);
	return IRQ_HANDLED;
}

/* Return TIOCSER_TEMT when transmitter is not busy
 * or query the tx fifo is not empty? */
static u_int wk2xxx_tx_empty(struct uart_port *port)
{
	uint8_t fsr = 0;
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	mutex_lock(&wk2xxxs_lock);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FSR_REG, &fsr);

	while ((fsr & WK2XXX_FSR_TDAT_BIT) | (fsr & WK2XXX_FSR_TBUSY_BIT)) {
		wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FSR_REG,
				      &fsr);
	}

	priv->tx_empty =
		((fsr & (WK2XXX_FSR_TBUSY_BIT | WK2XXX_FSR_TDAT_BIT)) == 0) //
			?
			TIOCSER_TEMT //
			:
			0;
	mutex_unlock(&wk2xxxs_lock);

	dev_dbg(&spi->dev, "%s: exit port=%ld tx_empty=0x%x fsr=0x%x\n",
		__func__, one->port.iobase, priv->tx_empty, fsr);

	return priv->tx_empty;
}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called. mctrl=0x%x\n", __func__, mctrl);
}
static u_int wk2xxx_get_mctrl(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called.\n", __func__);

	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void wk2xxx_stop_tx(struct uart_port *port) //
{
	uint8_t sier;
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	mutex_lock(&wk2xxxs_lock);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, &sier);

	sier &= ~WK2XXX_SIER_TFTRIG_IEN_BIT;

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, sier);
	mutex_unlock(&wk2xxxs_lock);

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static void wk2xxx_start_tx_proc(struct kthread_work *ws)
{
	struct wk2xxx_one *one = to_wk2xxx_one(ws, start_tx_work);
	struct uart_port *port = &(to_wk2xxx_one(ws, start_tx_work)->port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	uint8_t rx;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	mutex_lock(&wk2xxxs_lock);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, &rx);

	rx |= WK2XXX_SIER_TFTRIG_IEN_BIT | WK2XXX_SIER_RFTRIG_IEN_BIT |
	      WK2XXX_SIER_RXOUT_IEN_BIT;

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, rx);
	mutex_unlock(&wk2xxxs_lock);
}

/*
 *  * 
*/
static void wk2xxx_start_tx(struct uart_port *port)
{
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	bool ret;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

#ifdef WK_WORK_KTHREAD
	ret = kthread_queue_work(&priv->kworker, &one->start_tx_work);
#else
	ret = queue_kthread_work(&priv->kworker, &one->start_tx_work);
#endif

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static void wk2xxx_stop_rx_proc(struct kthread_work *ws)
{
	struct wk2xxx_one *one = to_wk2xxx_one(ws, stop_rx_work);
	struct uart_port *port = &(to_wk2xxx_one(ws, stop_rx_work)->port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	uint8_t rx;

	dev_dbg(&spi->dev, "%s: enter port:%ld\n", __func__, one->port.iobase);

	mutex_lock(&wk2xxxs_lock);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, &rx);

	rx &= ~WK2XXX_SIER_RFTRIG_IEN_BIT;
	rx &= ~WK2XXX_SIER_RXOUT_IEN_BIT;

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, rx);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, &rx);

	rx &= ~WK2XXX_SCR_RXEN_BIT;
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, rx);
	mutex_unlock(&wk2xxxs_lock);

	dev_dbg(&spi->dev, "%s: exit port:%ld\n", __func__, one->port.iobase);
}

static void wk2xxx_stop_rx(struct uart_port *port)
{
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	bool ret;

	dev_dbg(&spi->dev, "%s: enter port:%ld\n", __func__, one->port.iobase);

#ifdef WK_WORK_KTHREAD
	ret = kthread_queue_work(&priv->kworker, &one->stop_rx_work);
#else
	ret = queue_kthread_work(&priv->kworker, &one->stop_rx_work);
#endif

	dev_dbg(&spi->dev, "%s: exit port:%ld\n", __func__, one->port.iobase);
}

/*
 *  * No modem control lines
 *   */
static void wk2xxx_enable_ms(struct uart_port *port) //nothing
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called.\n", __func__);
}
/*
 *  * Interrupts always disabled.
*/
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called. break_state=0x%x\n", __func__,
		break_state);
}

static int wk2xxx_startup(struct uart_port *port) //i
{
	uint8_t gena, grst, gier, sier, scr, dat[1];
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: enter port:%ld\n", __func__, one->port.iobase);
	dev_dbg(&spi->dev, "%s: port1=%ld port2=%ld port3=%ld port4=%ld\n",
		__func__, priv->p[0].port.iobase, priv->p[1].port.iobase,
		priv->p[2].port.iobase, priv->p[3].port.iobase);
	dev_dbg(&spi->dev, "%s: line1=%d line2=%d line3=%d line4=%d\n",
		__func__, priv->p[0].line, priv->p[1].line, priv->p[2].line,
		priv->p[3].line);

	mutex_lock(&wk2xxxs_global_lock);
	wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
	gena = dat[0];
	switch (one->port.iobase) {
	case 1:
		gena |= WK2XXX_GENA_UT1EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 2:
		gena |= WK2XXX_GENA_UT2EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 3:
		gena |= WK2XXX_GENA_UT3EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 4:
		gena |= WK2XXX_GENA_UT4EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	default:
		dev_warn(&spi->dev, "%s: (1) bad iobase: %d.\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	//wk2xxx_read_global_reg(spi, WK2XXX_GRST_REG, dat);
	grst = 0;
	switch (one->port.iobase) {
	case 1:
		grst |= WK2XXX_GRST_UT1RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 2:
		grst |= WK2XXX_GRST_UT2RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 3:
		grst |= WK2XXX_GRST_UT3RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 4:
		grst |= WK2XXX_GRST_UT4RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	default:
		dev_warn(&spi->dev, "%s: (2) bad iobase: %d.\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	//enable the sub port interrupt
	wk2xxx_read_global_reg(spi, WK2XXX_GIER_REG, dat);
	gier = dat[0];
	switch (one->port.iobase) {
	case 1:
		gier |= WK2XXX_GIER_UT1IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 2:
		gier |= WK2XXX_GIER_UT2IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 3:
		gier |= WK2XXX_GIER_UT3IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 4:
		gier |= WK2XXX_GIER_UT4IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	default:
		dev_warn(&spi->dev, "%s: (3) bad iobase: %d.\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, dat);
	sier = dat[0];
	sier &= ~WK2XXX_SIER_TFTRIG_IEN_BIT;
	sier |= WK2XXX_SIER_RFTRIG_IEN_BIT;
	sier |= WK2XXX_SIER_RXOUT_IEN_BIT;
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, sier);

	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, dat);
	scr = dat[0] | WK2XXX_SCR_TXEN_BIT | WK2XXX_SCR_RXEN_BIT;
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, scr);

	//initiate the fifos
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FCR_REG,
			       0xff); //initiate the fifos
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FCR_REG, 0xfc);
	//set rx/tx interrupt
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 1);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_RFTL_REG,
			       WK2XXX_RXFIFO_LEVEL);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_TFTL_REG,
			       WK2XXX_TXFIFO_LEVEL);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 0);

/*enable rs485*/
#ifdef WK_RS485_FUNCTION
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_RS485_REG,
			       0X02); //default  high
	//wk2xxx_write_slave_reg(s->spi_wk,one->port.iobase,WK2XXX_RS485,0X03);//default low
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 0X01);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_RRSDLY_REG, 0X10);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 0X00);
#endif

	/*****************************test**************************************/
#ifdef _DEBUG_WK_TEST
	wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, &gena);
	wk2xxx_read_global_reg(spi, WK2XXX_GIER_REG, &gier);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, &sier);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, &scr);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FCR_REG, dat);
	dev_dbg(&spi->dev,
		"%s: port=%ld gena=0x%x gier=0x%x sier=0x%x scr=0x%x fcr=0x%x\n",
		__func__, one->port.iobase, gena, gier, sier, scr, dat[0]);
#endif
	/**********************************************************************/

	mutex_unlock(&wk2xxxs_global_lock);
	uart_circ_clear(&one->port.state->xmit);
	wk2xxx_enable_ms(&one->port);

	dev_dbg(&spi->dev, "%s: exit port:%ld\n", __func__, one->port.iobase);

	return 0;
}

static void wk2xxx_shutdown(struct uart_port *port)
{
	uint8_t gena, grst, gier, dat[1];
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	mutex_lock(&wk2xxxs_global_lock);
	wk2xxx_read_global_reg(spi, WK2XXX_GIER_REG, &gier);
	switch (one->port.iobase) {
	case 1:
		gier &= ~WK2XXX_GIER_UT1IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 2:
		gier &= ~WK2XXX_GIER_UT2IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 3:
		gier &= ~WK2XXX_GIER_UT3IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	case 4:
		gier &= ~WK2XXX_GIER_UT4IE_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GIER_REG, gier);
		break;
	default:
		dev_warn(&spi->dev, "%s: (1) (GIER) bad iobase %d\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, 0x0);
	mutex_unlock(&wk2xxxs_global_lock);

#ifdef WK_WORK_KTHREAD
	kthread_flush_work(&one->start_tx_work);
	kthread_flush_work(&one->stop_rx_work);
	kthread_flush_work(&priv->irq_work);
	//kthread_flush_worker(&priv->kworker);
#else
	flush_kthread_work(&one->start_tx_work);
	flush_kthread_work(&one->stop_rx_work);
	flush_kthread_work(&priv->irq_work);
	//flush_kthread_worker(&priv->kworker);
#endif

	mutex_lock(&wk2xxxs_global_lock);
	wk2xxx_read_global_reg(spi, WK2XXX_GRST_REG, dat);
	grst = dat[0];
	switch (one->port.iobase) {
	case 1:
		grst |= WK2XXX_GRST_UT1RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 2:
		grst |= WK2XXX_GRST_UT2RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 3:
		grst |= WK2XXX_GRST_UT3RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	case 4:
		grst |= WK2XXX_GRST_UT4RST_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GRST_REG, grst);
		break;
	default:
		dev_warn(&spi->dev, "%s: (2) bad iobase %d\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
	gena = dat[0];
	switch (one->port.iobase) {
	case 1:
		gena &= ~WK2XXX_GENA_UT1EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 2:
		gena &= ~WK2XXX_GENA_UT2EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 3:
		gena &= ~WK2XXX_GENA_UT3EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	case 4:
		gena &= ~WK2XXX_GENA_UT4EN_BIT;
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, gena);
		break;
	default:
		dev_warn(&spi->dev, "%s: (3) bad iobase %d\n", __func__,
			 (uint8_t)one->port.iobase);
		break;
	}

	mutex_unlock(&wk2xxxs_global_lock);

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static void conf_wk2xxx_subport(struct uart_port *port)
{
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	uint8_t sier = 0, fwcr = 0, lcr = 0, scr = 0, dat[1], baud0 = 0,
		baud1 = 0, pres = 0, count = 200;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);

	lcr = one->new_lcr_reg;
	//scr = priv->new_scr_reg;
	baud0 = one->new_baud0_reg;
	baud1 = one->new_baud1_reg;
	pres = one->new_pres_reg;
	fwcr = one->new_fwcr_reg;

	/* Disable Uart all interrupts */
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, dat);
	sier = dat[0];
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, 0X0);

	do {
		wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_FSR_REG,
				      dat);
	} while ((dat[0] & WK2XXX_FSR_TBUSY_BIT) && (count--));

	// then, disable tx and rx
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG, dat);
	scr = dat[0];
	wk2xxx_write_slave_reg(
		spi, one->port.iobase, WK2XXX_SCR_REG,
		scr & (~(WK2XXX_SCR_RXEN_BIT | WK2XXX_SCR_TXEN_BIT)));

	// set the parity, stop bits and data size //
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_LCR_REG, lcr);

#ifdef WK_FLOWCTRL_FUNCTION
	if (fwcr > 0) {
		dev_dbg(&spi->dev, "%s: [Flow Control] fwcr=0x%X\n", __func__,
			fwcr);
		// Configure flow control levels
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FWCR_REG,
				       fwcr);
		//Flow control halt level 0XF0, resume level 0X80
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG,
				       1);
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FWTH_REG,
				       0XF0);
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_FWTL_REG,
				       0X80);
		wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG,
				       0);
	}
#endif

	/* Setup baudrate generator */
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 1);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_BAUD0_REG, baud0);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_BAUD1_REG, baud1);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_PRES_REG, pres);

#ifdef _DEBUG_WK_FUNCTION
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_BAUD0_REG, &baud1);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_BAUD1_REG, &baud0);
	wk2xxx_read_slave_reg(spi, one->port.iobase, WK2XXX_PRES_REG, &pres);
	dev_dbg(&spi->dev, "%s: baud0=0x%x baud1=0x%x pres=0x%x\n", __func__,
		baud0, baud1, pres);
#endif

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SPAGE_REG, 0);
	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SCR_REG,
			       scr | (WK2XXX_SCR_RXEN_BIT |
				      WK2XXX_SCR_TXEN_BIT));

	wk2xxx_write_slave_reg(spi, one->port.iobase, WK2XXX_SIER_REG, sier);

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static void wk2xxx_termios(struct uart_port *port, struct ktermios *termios,
			   struct ktermios *old)
{
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	uint8_t lcr = 0, fwcr = 0, baud1 = 0, baud0 = 0, pres = 0,
		bParityType = 0;
	uint32_t temp = 0, freq = 0;
	int baud = 0;

	dev_dbg(&spi->dev, "%s: enter port=%ld\n", __func__, one->port.iobase);
	dev_dbg(&spi->dev, "%s: termios.c_cflag=0x%x termios.c_iflag=0x%x\n",
		__func__, termios->c_cflag, termios->c_iflag);

	baud = tty_termios_baud_rate(termios);
	freq = one->port.uartclk;
	if (freq >= (baud * 16)) {
		temp = (freq) / (baud * 16);
		temp = temp - 1;
		baud1 = (uint8_t)((temp >> 8) & 0xff);
		baud0 = (uint8_t)(temp & 0xff);
		temp = (((freq % (baud * 16)) * 100) / (baud));
		pres = (temp + 100 / 2) / 100;
		dev_dbg(&spi->dev, "%s: baud0=%x baud1=%x pres=%x\n", __func__,
			baud0, baud1, pres);
		dev_dbg(&spi->dev, "%s: freq=%d baudrate=%d\n", __func__, freq,
			baud);
	} else {
		dev_err(&spi->dev, "%s: The baud rate %d is too high.\n",
			__func__, baud);
	}
	tty_termios_encode_baud_rate(termios, baud, baud);

	lcr = 0;
	if (termios->c_cflag & CSTOPB)
		lcr |= WK2XXX_LCR_STPL_BIT; //two  stop_bits
	else
		lcr &= ~WK2XXX_LCR_STPL_BIT; //one  stop_bits

	bParityType = termios->c_cflag & PARENB ?
			      (termios->c_cflag & PARODD ? 1 : 2) +
				      (termios->c_cflag & CMSPAR ? 2 : 0) :
			      0;
	if (termios->c_cflag & PARENB) {
		lcr |= WK2XXX_LCR_PAEN_BIT; //enbale spa
		switch (bParityType) {
		case 0x01: //ODD
			lcr |= WK2XXX_LCR_PAM0_BIT;
			lcr &= ~WK2XXX_LCR_PAM1_BIT;
			break;
		case 0x02: //EVEN
			lcr |= WK2XXX_LCR_PAM1_BIT;
			lcr &= ~WK2XXX_LCR_PAM0_BIT;
			break;
		case 0x03: //MARK--1
			lcr |= WK2XXX_LCR_PAM1_BIT | WK2XXX_LCR_PAM0_BIT;
			break;
		case 0x04: //SPACE--0
			lcr &= ~WK2XXX_LCR_PAM1_BIT;
			lcr &= ~WK2XXX_LCR_PAM0_BIT;
			break;
		default:
			lcr &= ~WK2XXX_LCR_PAEN_BIT;
			break;
		}
	}

	/* Set read status mask */
	port->read_status_mask = WK2XXX_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= WK2XXX_LSR_PE_BIT | WK2XXX_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= WK2XXX_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= WK2XXX_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= WK2XXX_LSR_BRK_ERROR_MASK;

#ifdef WK_FLOWCTRL_FUNCTION
	/* Configure flow control */
	if (termios->c_cflag & CRTSCTS) {
		fwcr = 0X30;
		dev_dbg(&spi->dev,
			"%s: [CRTSCTS] port=%lx lcr=0x%x fwcr=0x%x\n", __func__,
			one->port.iobase, lcr, fwcr);
	}

	if (termios->c_iflag & IXON) {
		dev_dbg(&spi->dev, "%s: [IXON] c_cflag=0x%x\n", __func__,
			termios->c_cflag);
	}
	if (termios->c_iflag & IXOFF) {
		dev_dbg(&spi->dev, "%s: [IXOFF] c_cflag=0x%x\n", __func__,
			termios->c_cflag);
	}
#endif

	one->new_baud1_reg = baud1;
	one->new_baud0_reg = baud0;
	one->new_pres_reg = pres;
	one->new_lcr_reg = lcr;
	one->new_fwcr_reg = fwcr;

#ifdef _DEBUG_WK_VALUE
	dev_dbg(&spi->dev, "%s: port=%lx lcr=0x%x fwcr=0x%x\n", __func__,
		one->port.iobase, lcr, fwcr);
#endif

	conf_wk2xxx_subport(&one->port);

	dev_dbg(&spi->dev, "%s: exit port=%ld\n", __func__, one->port.iobase);
}

static const char *wk2xxx_type(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called. port type=%d\n", __func__, port->type);

	return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;
}

static void wk2xxx_release_port(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called.\n", __func__);
}

static int wk2xxx_request_port(struct uart_port *port)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;

	dev_dbg(&spi->dev, "%s: called.\n", __func__);
	return 0;
}

static void wk2xxx_config_port(struct uart_port *port, int flags)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	struct wk2xxx_one *one = to_wk2xxx_one(port, port);

	dev_dbg(&spi->dev, "%s: called. flags=0x%x\n", __func__, flags);

	if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
		one->port.type = PORT_WK2XXX;
}

static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct wk2xxx_port *priv = dev_get_drvdata(port->dev);
	struct spi_device *spi = priv->spi_wk;
	int ret = 0;

	dev_dbg(&spi->dev, "%s: called. user=0x%p\n", __func__, ser);

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_PORT)
		ret = -EINVAL;
	//if (port->uartclk / 16 != ser->baud_base)
	//     ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops wk2xxx_pops = {
	tx_empty: wk2xxx_tx_empty,
	set_mctrl: wk2xxx_set_mctrl,
	get_mctrl: wk2xxx_get_mctrl,
	stop_tx: wk2xxx_stop_tx,
	start_tx: wk2xxx_start_tx,
	stop_rx: wk2xxx_stop_rx,
	enable_ms: wk2xxx_enable_ms,
	break_ctl: wk2xxx_break_ctl,
	startup: wk2xxx_startup,
	shutdown: wk2xxx_shutdown,
	set_termios: wk2xxx_termios,
	type: wk2xxx_type,
	release_port: wk2xxx_release_port,
	request_port: wk2xxx_request_port,
	config_port: wk2xxx_config_port,
	verify_port: wk2xxx_verify_port,

};
static struct uart_driver wk2xxx_uart_driver = {
	owner: THIS_MODULE,
	major: SERIAL_WK2XXX_MAJOR,
	driver_name: "ttySWK",
	dev_name: "ttysWK",
	minor: MINOR_START,
	nr: WK2_NR_PORTS,
	cons: NULL
};

static int uart_driver_registered;
static struct spi_driver wk2xxx_driver;

#ifdef WK_RSTGPIO_FUNCTION
static int wk2xxx_reset(struct device *dev)
{
	struct gpio_desc *rst;

	rst = devm_gpiod_get_optional(dev, WK2XXX_GPIO_NAME_RESET,
				      GPIOD_OUT_HIGH);
	if (IS_ERR(rst))
		return PTR_ERR(rst);

#ifdef WK_RST_GPIO_SLEEP
	gpiod_set_value_cansleep(rst, 0);
	mdelay(20);
	gpiod_set_value_cansleep(rst, 1);
	mdelay(10);
#else
	mutex_lock(&wk2xxxs_global_lock);
	gpiod_set_value(rst, 0);
	mdelay(20);
	gpiod_set_value(rst, 1);
	mdelay(10);
	mutex_unlock(&wk2xxxs_global_lock);
#endif

	dev_dbg(dev, "%s: Chip reset complete\n", __func__);
	return 0;
}

static int wk2xxx_spi_rstgpio_parse_dt(struct device *dev)
{
	struct wk2xxx_port *priv = dev_get_drvdata(dev);
	enum of_gpio_flags rst_flags;
	int ret;

	dev_dbg(dev, "%s: enter\n", __func__);

	ret = of_get_named_gpio_flags(dev->of_node, WK2XXX_GPIO_NAME_RESET, 0,
				      &rst_flags);
	if (!gpio_is_valid(ret) || ret == 0) {
		dev_err(dev, "%s: Can't find gpio '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_RESET, ret);
		return -ENOSYS;
	}

	/* Save reset gpio number */
	priv->rst_gpio_num = ret;

	ret = gpio_request(priv->rst_gpio_num, WK2XXX_GPIO_NAME_RESET);
	if (ret) {
		dev_err(dev, "%s: Can't get gpio '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_POWER, ret);
		goto error_gpio;
	}

	/* Set output with active high */
	gpio_direction_output(priv->rst_gpio_num, 1);
	mdelay(50);

	dev_dbg(dev, "%s: Using reset gpio=%d\n", __func__, priv->rst_gpio_num);

	/* Trigger reset */
	ret = wk2xxx_reset(dev);
	if (ret) {
		dev_err(dev, "%s: Unable to reset chip. error=%d\n", __func__,
			ret);
		goto error_gpio;
	}

	return 0;

error_gpio:
	gpio_free(priv->rst_gpio_num);
	priv->rst_gpio_num = 0;
	return -ENODEV;
}
#endif

#ifdef WK_CSGPIO_FUNCTION
static int wk2xxx_spi_csgpio_parse_dt(struct device *dev)
{
	struct wk2xxx_port *priv = dev_get_drvdata(dev);
	enum of_gpio_flags flags;
	int ret;

	dev_dbg(dev, "%s: enter\n", __func__);

	ret = of_get_named_gpio_flags(dev->of_node, WK2XXX_GPIO_NAME_CS, 0,
				      &flags);
	if (!gpio_is_valid(ret) || ret == 0) {
		dev_err(dev, "%s: Can't find gpio '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_CS, ret);
		return -ENOSYS;
	}

	/* Save gpio number for later */
	priv->cs_gpio_num = ret;

	if (gpio_request(priv->cs_gpio_num, WK2XXX_GPIO_NAME_CS)) {
		dev_err(dev, "%s: Can't get gpio '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_CS, ret);
		goto error_gpio;
	}

	/* Set gpio output with active high */
	gpio_direction_output(priv->cs_gpio_num, 1);

	dev_dbg(dev, "%s: Using CS gpio=%d\n", __func__, priv->cs_gpio_num);
	return 0;

error_gpio:
	gpio_free(priv->cs_gpio_num);
	priv->cs_gpio_num = 0;
	return -ENODEV;
}
#endif

#ifdef WK_PWRGPIO_FUNCTION
static int wk2xxx_spi_pwrgpio_parse_dt(struct device *dev)
{
	struct wk2xxx_port *priv = dev_get_drvdata(dev);
	enum of_gpio_flags flags;
	int ret;

	dev_dbg(dev, "%s: start\n", __func__);

	ret = of_get_named_gpio_flags(dev->of_node, WK2XXX_GPIO_NAME_POWER, 0,
				      &flags);
	if (!gpio_is_valid(ret) || ret == 0) {
		dev_err(dev, "%s: Can't find OF '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_POWER, ret);
		return -ENOSYS;
	}

	/* save gpio number for late */
	priv->pwr_gpio_num = ret;

	ret = gpio_request(priv->pwr_gpio_num, WK2XXX_GPIO_NAME_POWER);
	if (ret) {
		dev_err(dev, "%s: Can't get gpio '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_POWER, ret);
		goto error_gpio;
	}

	/* set output with active high */
	gpio_direction_output(priv->pwr_gpio_num, 1);

	dev_dbg(dev, "%s: Using power gpio=%d\n", __func__, priv->pwr_gpio_num);
	return 0;

error_gpio:
	gpio_free(priv->pwr_gpio_num);
	priv->pwr_gpio_num = 0;
	return -ENODEV;
}
#endif

static int wk2xxx_spi_irq_parse_dt(struct device *dev)
{
	struct wk2xxx_port *priv = dev_get_drvdata(dev);
	enum of_gpio_flags flags;
	int ret, irq;

	dev_dbg(dev, "%s: enter\n", __func__);

	ret = of_get_named_gpio_flags(dev->of_node, WK2XXX_GPIO_NAME_IRQ, 0,
				      &flags);
	if (!gpio_is_valid(ret) || ret == 0) {
		dev_err(dev, "%s: Can't find OF '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_IRQ, ret);
		return -ENOSYS;
	}

	priv->irq_gpio_num = ret;

	irq = gpio_to_irq(priv->irq_gpio_num);
	if (irq) {
		ret = gpio_request(priv->irq_gpio_num, WK2XXX_GPIO_NAME_IRQ);
		if (ret) {
			dev_err(dev, "%s: Can't get gpio '%s'. error=%d\n",
				__func__, WK2XXX_GPIO_NAME_IRQ, ret);
			ret = IRQ_NONE;
			goto error_gpio;
		}
	} else {
		dev_err(dev, "%s: Can't get irq of '%s'. error=%d\n", __func__,
			WK2XXX_GPIO_NAME_IRQ, ret);
		ret = -ENODEV;
		goto error_gpio;
	}

	priv->irq_num = irq;

	dev_dbg(dev, "%s: Using irq=%d gpio=%d\n", __func__, priv->irq_num,
		priv->irq_gpio_num);
	return 0;

error_gpio:
	gpio_free(priv->irq_gpio_num);
	priv->irq_gpio_num = 0;
	return ret;
}

static int wk2xxx_probe(struct spi_device *spi)
{
	const struct sched_param sched_param = {
		.sched_priority = MAX_RT_PRIO / 2,
	};

	struct wk2xxx_port *priv;
	uint8_t dat[1];
	uint8_t i;
	int ret, priv_size;

	dev_info(&spi->dev, DRIVER_DESC "\n");
	dev_info(&spi->dev, VERSION_DESC "\n");

	dev_dbg(&spi->dev, "%s: Setup spi device.\n", __func__);
	
	/*
	 * TODO: get spi parameters from DT 
	 */

	/* Setup SPI bus */
	spi->bits_per_word = 8;

	/* only supports mode 0 on WK2124 */
	spi->mode = spi->mode ?: SPI_MODE_0;
	spi->max_speed_hz = spi->max_speed_hz ?: 10000000;

	ret = spi_setup(spi);
	if (ret)
		return ret;

	priv_size = sizeof(struct wk2xxx_port) +
		    sizeof(struct wk2xxx_one) * WK2_NR_PORTS;

	/* Alloc port structure */
	priv = devm_kzalloc(&spi->dev, priv_size, GFP_KERNEL);
	if (IS_ERR(priv)) {
		dev_err(&spi->dev, "%s: Out of memory.\n", __func__);
		return PTR_ERR(priv);
	}

	/* Clear private data */
	memset(priv, 0, priv_size);

	/* Link spi device to private driver data for later use */
	priv->spi_wk = spi;

	/* Set chip type descriptor 
	 * TODO: get type from DT */
	priv->devtype = &wk2124_devtype;

	/* Assign private driver data to device */
	dev_set_drvdata(&spi->dev, priv);

#ifdef WK_PWRGPIO_FUNCTION
	/* Obtain the GPIO number of power signal */
	ret = wk2xxx_spi_pwrgpio_parse_dt(&spi->dev);
	if (ret) {
		dev_err(&spi->dev, "%s: Unable to get power gpio. error=%d\n",
			__func__, ret);
		goto error_gpio;
	}
#endif

#ifdef WK_RSTGPIO_FUNCTION
	/* Obtain the GPIO number of RST signal */
	ret = wk2xxx_spi_rstgpio_parse_dt(&spi->dev);
	if (ret) {
		dev_err(&spi->dev, "%s: Unable to get reset gpio. error=%d\n",
			__func__, ret);
		goto error_gpio;
	}
#endif

#ifdef WK_CSGPIO_FUNCTION
	/* Obtain the GPIO number of CS signal */
	ret = wk2xxx_spi_csgpio_parse_dt(&spi->dev);
	if (ret) {
		dev_err(&spi->dev, "%s: Unable to get CS gpio. error=%d\n",
			__func__, ret);
		goto error_gpio;
	}
#endif

	/* Obtain the IRQ signal GPIO number and interrupt number */
	ret = wk2xxx_spi_irq_parse_dt(&spi->dev);
	if (ret) {
		dev_err(&spi->dev, "%s: Unable to get irq. error=%d\n",
			__func__, ret);
		goto error_gpio;
	}

	/* Test spi function: chip answer GENA => 0x30 0x35 0x3f */
	do {
		wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
		wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
		dev_dbg(&spi->dev, "%s: Chip reg read GENA=0x%x\n", __func__,
			dat[0]);
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xf5);
		wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
		dev_dbg(&spi->dev, "%s: Chip fnc=0xf5 GENA=0x%x\n", __func__,
			dat[0]);
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xff);
		wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
		dev_dbg(&spi->dev, "%s: Chip fnc=0xff GENA=0x%x\n", __func__,
			dat[0]);
		wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xf0);
	} while (0);

	/* Get interrupt number */
	wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0x0);
	wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
	if ((dat[0] & 0xf0) != 0x30) {
		dev_err(&spi->dev,
			"%s: The spi failed to read the register.%d\n",
			__func__, ret);
		ret = -ENODEV;
		goto error_gpio;
	}

	/*Init kthread_worker and kthread_work */
#ifdef WK_WORK_KTHREAD
	kthread_init_worker(&(priv->kworker));
	kthread_init_work(&priv->irq_work, wk2xxx_ist);
#else
	init_kthread_worker(&(priv->kworker));
	init_kthread_work(&priv->irq_work, wk2xxx_ist);
#endif

	priv->kworker_task =
		kthread_run(kthread_worker_fn, &priv->kworker, "wk2xxx");
	if (IS_ERR(priv->kworker_task)) {
		ret = PTR_ERR(priv->kworker_task);
		dev_err(&spi->dev,
			"%s: Unable to initialize kthread worker. error=%d\n",
			__func__, ret);
		goto out_clk;
	}
	sched_setscheduler(priv->kworker_task, SCHED_FIFO, &sched_param);

	/**/
	mutex_lock(&wk2xxxs_lock);
	if (!uart_driver_registered) {
		uart_driver_registered = 1;
		ret = uart_register_driver(&wk2xxx_uart_driver);
		if (ret) {
			dev_err(&spi->dev,
				"%s: Unable to register uart driver. error=%d\n",
				__func__, ret);
			mutex_unlock(&wk2xxxs_lock);
			goto out_clk;
		}
	}

	dev_dbg(&spi->dev, "%s: Initialize serial ports.\n", __func__);

	for (i = 0; i < WK2_NR_PORTS; i++) {
		priv->p[i].line = i;
		priv->p[i].port.dev = &spi->dev;
		priv->p[i].port.line = i;
		priv->p[i].port.ops = &wk2xxx_pops;
		priv->p[i].port.uartclk = WK_CRASTAL_CLK;
		priv->p[i].port.fifosize = 256;
		priv->p[i].port.iobase = i + 1;
		//priv->p[i].port.irq      = irq;
		priv->p[i].port.iotype = SERIAL_IO_PORT;
		priv->p[i].port.flags = UPF_BOOT_AUTOCONF;
		//priv->p[i].port.flags    = ASYNC_BOOT_AUTOCONF;
		//priv->p[i].port.iotype   = UPIO_PORT;
		//priv->p[i].port.flags    = UPF_FIXED_TYPE | UPF_LOW_LATENCY;

#ifdef WK_WORK_KTHREAD
		kthread_init_work(&priv->p[i].start_tx_work,
				  wk2xxx_start_tx_proc);
		kthread_init_work(&priv->p[i].stop_rx_work,
				  wk2xxx_stop_rx_proc);
#else
		init_kthread_work(&priv->p[i].start_tx_work,
				  wk2xxx_start_tx_proc);
		init_kthread_work(&priv->p[i].stop_rx_work,
				  wk2xxx_stop_rx_proc);
#endif
		/* Register uart port */
		ret = uart_add_one_port(&wk2xxx_uart_driver, &priv->p[i].port);
		if (ret < 0) {
			dev_err(&spi->dev,
				"%s: Unable to add port for line i=%d with error=%d\n",
				__func__, i, ret);
			mutex_unlock(&wk2xxxs_lock);
			goto out_port;
		}

		dev_dbg(&spi->dev, "%s: uart port=%ld registred. ret=0x%d\n",
			__func__, priv->p[i].port.iobase, ret);
	}
	mutex_unlock(&wk2xxxs_lock);

	dev_dbg(&spi->dev, "%s: Setup interrupt.\n", __func__);

	/* Setup interrupt */
	ret = devm_request_irq(&spi->dev, priv->irq_num, wk2xxx_irq,
			       IRQF_TRIGGER_FALLING, dev_name(&spi->dev), priv);
	if (ret) {
		dev_err(&spi->dev, "%s: Unable to request irq=%d error=%d.\n",
			__func__, priv->irq_num, ret);
		goto out_port;
	}

	/* probe successfully completed */
	priv->init_done = 1;

	dev_info(&spi->dev, "Driver successfully installed.\n");
	return 0;

out_port:
	for (i = 0; i < WK2_NR_PORTS; i++) {
		uart_remove_one_port(&wk2xxx_uart_driver, &priv->p[i].port);
	}

out_clk:
	kthread_stop(priv->kworker_task);

error_gpio:
	if (priv->irq_gpio_num > 0) {
		gpio_free(priv->irq_gpio_num);
		priv->irq_gpio_num = 0;
	}

#ifdef WK_RSTGPIO_FUNCTION
	if (priv->rst_gpio_num > 0) {
		gpio_free(priv->rst_gpio_num);
		priv->rst_gpio_num = 0;
	}
#endif

#ifdef WK_CSGPIO_FUNCTION
	if (priv->cs_gpio_num > 0) {
		gpio_free(priv->cs_gpio_num);
		priv->cs_gpio_num = 0;
	}
#endif

#ifdef WK_PWRGPIO_FUNCTION
	if (priv->pwr_gpio_num > 0) {
		gpio_free(priv->pwr_gpio_num);
		priv->pwr_gpio_num = 0;
	}
#endif

	/* skip driver remove on unload */
	priv->init_done = 0;

	dev_err(&spi->dev, "%s: Driver probe failed. error=%d\n", __func__,
		ret);
	return ret;
}

static int wk2xxx_remove(struct spi_device *spi)
{
	struct wk2xxx_port *priv = dev_get_drvdata(&spi->dev);
	int i;

	if (!priv)
		return 0;

	if (!priv->init_done) {
		devm_kfree(&spi->dev, priv);
		return 0;
	}

	dev_dbg(&spi->dev, "%s: Driver cleanup\n", __func__);

	mutex_lock(&wk2xxxs_lock);

	/* release user space ports */
	for (i = 0; i < WK2_NR_PORTS; i++) {
		uart_remove_one_port(&wk2xxx_uart_driver, &priv->p[i].port);
	}

#ifdef WK_WORK_KTHREAD
	kthread_flush_worker(&priv->kworker);
#else
	flush_kthread_worker(&priv->kworker);
#endif

	/* stop worker thread */
	kthread_stop(priv->kworker_task);

	/* release gpio's */
	if (priv->irq_gpio_num > 0) {
		gpio_free(priv->irq_gpio_num);
		priv->irq_gpio_num = 0;
	}

#ifdef WK_CSGPIO_FUNCTION
	if (priv->cs_gpio_num > 0) {
		gpio_free(priv->cs_gpio_num);
		priv->cs_gpio_num = 0;
	}
#endif

#ifdef WK_RSTGPIO_FUNCTION
	if (priv->rst_gpio_num > 0) {
		gpio_free(priv->rst_gpio_num);
		priv->rst_gpio_num = 0;
	}
#endif

#ifdef WK_PWRGPIO_FUNCTION
	if (priv->pwr_gpio_num > 0) {
		gpio_free(priv->pwr_gpio_num);
		priv->pwr_gpio_num = 0;
	}
#endif

	uart_unregister_driver(&wk2xxx_uart_driver);
	mutex_unlock(&wk2xxxs_lock);

	devm_kfree(&spi->dev, priv);

	dev_info(&spi->dev, "Driver removed\n");
	return 0;
}

static const struct of_device_id wkmic_spi_dt_match[] = {
	{
		.compatible = "wkmic,wk2124_spi",
	},
	{
		.compatible = "firefly,spi-wk2xxx",
	},
	{},
};

MODULE_DEVICE_TABLE(of, wkmic_spi_dt_match);

static struct spi_driver wk2xxx_driver = {
    .driver = {
		.name           = "wk2xxxspi",
		.bus            = &spi_bus_type,
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(wkmic_spi_dt_match),
    },
    .probe  = wk2xxx_probe,
    .remove = wk2xxx_remove,
};

static int __init wk2xxx_init(void)
{
	return spi_register_driver(&wk2xxx_driver);
}

static void __exit wk2xxx_exit(void)
{
	return spi_unregister_driver(&wk2xxx_driver);
}

module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
