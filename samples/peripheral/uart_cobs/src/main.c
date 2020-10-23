/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <uart_cobs.h>


#define SW0 0
#define SW1 1
#define MAX_MSG_SIZE 64


enum user_state {
	USER_STATE_NONE,
	USER_STATE_PONG,
	USER_STATE_PING
};

struct user_info {
	struct uart_cobs_user user;
	const char *name;
	enum user_state state;
};


static char rx_data[MAX_MSG_SIZE + 1];
static char tx_data[MAX_MSG_SIZE + 1];

const char user_a_name[] = "A";
const char user_b_name[] = "B";

static void cobs_idle_evt_handler(const struct uart_cobs_user *user,
				  const struct uart_cobs_evt *evt);
static void cobs_user_evt_handler(const struct uart_cobs_user *user,
				  const struct uart_cobs_evt *evt);

UART_COBS_USER_DEFINE(cobs_idle, cobs_idle_evt_handler);
static struct user_info user_a = {
	.user = {
		.cb = cobs_user_evt_handler
	},
	.name = user_a_name,
	.state = USER_STATE_NONE
};
static struct user_info user_b = {
	.user = {
		.cb = cobs_user_evt_handler
	},
	.name = user_b_name,
	.state = USER_STATE_NONE
};

static struct device *gpiob;
static struct gpio_callback gpio_cb;

static struct k_poll_signal sig_ready = K_POLL_SIGNAL_INITIALIZER(sig_ready);
static struct k_poll_signal sig_gpio = K_POLL_SIGNAL_INITIALIZER(sig_gpio);


static void log_error(const char *op, enum uart_cobs_err err)
{
	switch (err) {
	case UART_COBS_ERR_ABORT:
		printk("%s: aborted.", op);
		break;
	case UART_COBS_ERR_TIMEOUT:
		printk("%s: timed out.", op);
		break;
	case UART_COBS_ERR_BREAK:
		printk("%s: UART break error.", op);
		break;
	default:
		break;
	}
}

static bool msg_ping_start(struct user_info *info)
{
	info->state = USER_STATE_PING;
	int err = uart_cobs_user_start(&info->user);
	if (err == 0) {
		/* Start the exchange by sending a name to the other device. */
		uart_cobs_tx_buf_write(&info->user, info->name,
				strlen(info->name));
	} else {
		printk("Error %d starting %s\n", err, info->name);
		info->state = USER_STATE_NONE;
	}
	return err == 0;
}

static bool msg_pong_start(struct user_info *info)
{
	info->state = USER_STATE_PONG;
	int err = uart_cobs_user_start(&info->user);
	if (err != 0) {
		printk("Error %d starting %s\n", err, info->name);
		info->state = USER_STATE_NONE;
	}
	return err == 0;
}

static void cobs_idle_evt_handler(const struct uart_cobs_user *user,
				  const struct uart_cobs_evt *evt)
{
	switch (evt->type) {
	case UART_COBS_EVT_USER_START:
		__ASSERT(uart_cobs_rx_start(&cobs_idle, MAX_MSG_SIZE) == 0,
			 "Unable to start RX in idle state.");
		printk("Entered idle state.\n");
		k_poll_signal_raise(&sig_ready, 0);
		break;
	case UART_COBS_EVT_USER_END:
		k_poll_signal_reset(&sig_ready);
		printk("Exited idle state.\n");
		break;
	case UART_COBS_EVT_RX:
		/* Received data - check if we recognize it and switch user
		   if we do. */
		memcpy(rx_data, evt->data.rx.buf,
			MIN(evt->data.rx.len, MAX_MSG_SIZE));
		printk("Idle: received \"%s\"\n", rx_data);
		if (strncmp(rx_data, user_a.name, strlen(user_a.name)) == 0) {
			msg_pong_start(&user_a);
			
		} else if (strncmp(rx_data, user_b.name,
				   strlen(user_b.name)) == 0) {
			msg_pong_start(&user_b);
		}
		break;
	case UART_COBS_EVT_RX_ERR:
		log_error("RX", evt->data.err);
		break;
	case UART_COBS_EVT_TX_ERR:
		log_error("TX", evt->data.err);
		break;
	default:
		break;
	}
}

static void cobs_user_evt_handler(const struct uart_cobs_user *user,
				  const struct uart_cobs_evt *evt)
{
	struct user_info *info = CONTAINER_OF(user, struct user_info, user);

	switch (evt->type) {
	case UART_COBS_EVT_USER_START:
		printk("%s started!\n", info->name);
		uart_cobs_rx_start(user, MAX_MSG_SIZE);
		if (info->state == USER_STATE_PING) {
			/* Send ping. */
			snprintf(tx_data, MAX_MSG_SIZE,
				"Ping from %s!", info->name);
			uart_cobs_tx_buf_write(&info->user, tx_data,
					strnlen(tx_data, MAX_MSG_SIZE));
		}
		break;
	case UART_COBS_EVT_USER_END:
		info->state = USER_STATE_NONE;
		printk("%s exited with status %d.\n", info->name, evt->data.err);
		break;
	case UART_COBS_EVT_RX:
		memcpy(rx_data, evt->data.rx.buf,
			MIN(evt->data.rx.len, MAX_MSG_SIZE));
		printk("%s: received \"%s\"\n", info->name, rx_data);
		if (info->state == USER_STATE_PONG) {
			/* Received ping. */
			snprintf(tx_data, MAX_MSG_SIZE,
				"Pong from %s!", info->name);
			uart_cobs_tx_buf_write(&info->user, tx_data,
					strnlen(tx_data, MAX_MSG_SIZE));
		} else {
			/* Received pong. */
			(void) uart_cobs_user_end(user, 0);
		}
		break;
	case UART_COBS_EVT_TX:
		if (info->state == USER_STATE_PONG) {
			/* Sent pong */
			(void) uart_cobs_user_end(user, 0);
		}
		break;
	case UART_COBS_EVT_RX_ERR:
		log_error("RX", evt->data.err);
		(void) uart_cobs_user_end(user, evt->data.err);
		break;
	
	case UART_COBS_EVT_TX_ERR:
		log_error("TX", evt->data.err);
		(void) uart_cobs_user_end(user, evt->data.err);
		break;
	default:
		break;
	}
}

static void button_handler(struct device *gpiob, struct gpio_callback *cb,
			uint32_t pins)
{
	int button;
	switch (pins) {
	case BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios)):
		button = SW0;
		break;
	case BIT(DT_GPIO_PIN(DT_ALIAS(sw1), gpios)):
		button = SW1;
		break;
	default:
		return;	
	}
	k_poll_signal_raise(&sig_gpio, button);
}

static int buttons_init(void)
{
	gpiob = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));
	if (gpiob == 0) {
		printk("Nordic nRF GPIO driver was not found!\n");
		return 1;
	}
	int err = gpio_pin_configure(gpiob, DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
				GPIO_INPUT |
				DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios));
	if (err != 0) {
		goto done;
	}

	err = gpio_pin_configure(gpiob, DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
				GPIO_INPUT |
				DT_GPIO_FLAGS(DT_ALIAS(sw1), gpios));
	if (err != 0) {
		goto done;
	}

	err = gpio_pin_interrupt_configure(gpiob,
					DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
					GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		goto done;
	}

	err = gpio_pin_interrupt_configure(gpiob,
					DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
					GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		goto done;
	}
	
	gpio_init_callback(&gpio_cb, button_handler,
		BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) |
		BIT(DT_GPIO_PIN(DT_ALIAS(sw1), gpios)));
	err = gpio_add_callback(gpiob, &gpio_cb);
done:
	if (err != 0) {
		printk("Unable to configure SW0/SW1 GPIO pins!\n");
		return 1;
	}
	return 0;


}

static void sig_evt_reset(struct k_poll_event *evt)
{
	evt->signal->signaled = 0;
	evt->state = K_POLL_STATE_NOT_READY;
}

void main(void)
{
	printk("Starting UART COBS example.\n");

	k_poll_signal_init(&sig_ready);
	k_poll_signal_init(&sig_gpio);

	struct k_poll_event evt_ready = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sig_ready);
	struct k_poll_event evt_gpio = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sig_gpio);

	int err = buttons_init();
	if (err != 0) {
		printk("Unable to initialize buttons.\n");
		return;
	}

	err = uart_cobs_idle_user_set(&cobs_idle);
	if (err != 0) {
		printk("Unable to set UART COBS idle user.\n");
		return;
	}

	k_poll(&evt_ready, 1, K_FOREVER);
	sig_evt_reset(&evt_ready);

	printk("Press button 1 to ping as A\n");
	printk("Press button 1 to ping as B\n");

	for (;;) {
		k_poll(&evt_gpio, 1, K_FOREVER);
		int button = evt_gpio.signal->result;
		sig_evt_reset(&evt_ready);

		bool started = false;
		if (button == SW0) {
			started = msg_ping_start(&user_a);
		} else if (button == SW1) {
			started = msg_ping_start(&user_b);
		}
		if (started) {
			k_poll(&evt_ready, 1, K_FOREVER);
			sig_evt_reset(&evt_ready);
		}
	}
}
