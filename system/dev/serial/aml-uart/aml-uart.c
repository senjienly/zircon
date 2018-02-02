// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <threads.h>

#include <bits/limits.h>
#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/protocol/platform-bus.h>
#include <ddk/protocol/platform-defs.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/serial.h>
#include <hw/reg.h>
#include <soc/aml-common/aml-uart.h>

#include <zircon/assert.h>
#include <zircon/types.h>

#define CLK_XTAL    24000000

typedef struct {
    uint32_t port_num;
    pdev_vmo_buffer_t mmio;
    zx_handle_t irq_handle;
    thrd_t irq_thread;
    serial_state_cb notify_cb;
    void* cookie;
    // last state we sent to notify_cb
    uint32_t state;
    mtx_t lock;
} aml_uart_port_t;

typedef struct {
    serial_driver_protocol_t serial;
    zx_device_t* zxdev;
    aml_uart_port_t* ports;
    unsigned port_count;
} aml_uart_t;


static uint32_t read_state(aml_uart_port_t* port) {
    void* mmio = port->mmio.vaddr;
    volatile uint32_t* status_reg = mmio + AML_UART_STATUS;
    uint32_t status = readl(status_reg);

    mtx_lock(&port->lock);

    uint32_t state = 0;
    if (!(status & AML_UART_STATUS_RXEMPTY)) {
        state |= SERIAL_STATE_READABLE;
    }
    if (!(status & AML_UART_STATUS_TXFULL)) {
        state |= SERIAL_STATE_WRITABLE;
    }
    bool notify = (state != port->state);
    port->state = state;
    
    mtx_unlock(&port->lock);
    
    if (notify && port->notify_cb) {
        port->notify_cb(port->port_num, state, port->cookie);
    }

    return state;
}


static uint32_t aml_serial_get_port_count(void* ctx) {
    aml_uart_t* uart = ctx;
    return uart->port_count;
}

static zx_status_t aml_serial_config(void* ctx, uint32_t port_num, uint32_t baud_rate, uint32_t flags) {
    aml_uart_t* uart = ctx;
    if (port_num >= uart->port_count) {
        return ZX_ERR_INVALID_ARGS;
    }
//    aml_uart_port_t* uart_port = &uart->ports[port_num];
return ZX_OK;    
}

static zx_status_t aml_serial_read(void* ctx, uint32_t port_num, void* buf, size_t length,
                                   size_t* out_actual) {
    aml_uart_t* uart = ctx;
    if (port_num >= uart->port_count) {
        return ZX_ERR_INVALID_ARGS;
    }

    aml_uart_port_t* port = &uart->ports[port_num];
    void* mmio = port->mmio.vaddr;
    volatile uint32_t* rfifo_reg = mmio + AML_UART_RFIFO;

    uint8_t* bufptr = buf;
    uint8_t* end = bufptr + length;
// need to lock around reading status to avoid problems with aml_serial_write?

    while (bufptr < end && (read_state(port) & SERIAL_STATE_READABLE)) {
        uint32_t val = readl(rfifo_reg);
        *bufptr++ = val;
    }

    size_t read = (void *)bufptr - buf;
    if (read == 0) {
        return ZX_ERR_SHOULD_WAIT;
    }
    *out_actual = read;
    return ZX_OK;
}

static zx_status_t aml_serial_write(void* ctx, uint32_t port_num, const void* buf, size_t length,
                                    size_t* out_actual) {
    aml_uart_t* uart = ctx;
    if (port_num >= uart->port_count) {
        return ZX_ERR_INVALID_ARGS;
    }
    
    aml_uart_port_t* port = &uart->ports[port_num];
    void* mmio = port->mmio.vaddr;
    volatile uint32_t* wfifo_reg = mmio + AML_UART_WFIFO;


    const uint8_t* bufptr = buf;
    const uint8_t* end = bufptr + length;
    while (bufptr < end && (read_state(port) & SERIAL_STATE_WRITABLE)) {
        writel(*bufptr++, wfifo_reg);
    }

    size_t written = (void *)bufptr - buf;
    if (written == 0) {
        return ZX_ERR_SHOULD_WAIT;
    }
    *out_actual = written;
    return ZX_OK;
}

static zx_status_t aml_serial_set_notify_callback(void* ctx, uint32_t port_num, serial_state_cb cb,
                                                  void* cookie) {
    aml_uart_t* uart = ctx;
    if (port_num >= uart->port_count) {
        return ZX_ERR_INVALID_ARGS;
    }
    aml_uart_port_t* port = &uart->ports[port_num];

    port->notify_cb = cb;
    port->cookie = cookie;

    // this will trigger notifying current state
    read_state(port);

    return ZX_OK;
}

static serial_driver_ops_t aml_serial_ops = {
    .get_port_count = aml_serial_get_port_count,
    .config = aml_serial_config,
    .read = aml_serial_read,
    .write = aml_serial_write,
    .set_notify_callback = aml_serial_set_notify_callback,
};

static void aml_uart_release(void* ctx) {
    aml_uart_t* uart = ctx;
    for (unsigned i = 0; i < uart->port_count; i++) {
        aml_uart_port_t* port = &uart->ports[i];
// TODO cancel and thread join
        pdev_vmo_buffer_release(&port->mmio);
        zx_handle_close(port->irq_handle);
    }
    free(uart->ports);
    free(uart);
}

static zx_protocol_device_t uart_device_proto = {
    .version = DEVICE_OPS_VERSION,
    .release = aml_uart_release,
};

static int aml_uart_irq_thread(void *arg) {
    aml_uart_port_t* port = arg;

    void* mmio = port->mmio.vaddr;

    volatile uint32_t* ctrl_reg = mmio + AML_UART_CONTROL;
    volatile uint32_t* irq_ctrl_reg = mmio + AML_UART_IRQ_CONTROL;
    volatile uint32_t* reg5 = mmio + AML_UART_REG5;

    // reset the port
    uint32_t temp = readl(ctrl_reg);
    temp |= AML_UART_CONTROL_RSTRX | AML_UART_CONTROL_RSTTX | AML_UART_CONTROL_CLRERR;
    writel(temp, ctrl_reg);
    temp &= ~(AML_UART_CONTROL_RSTRX | AML_UART_CONTROL_RSTTX | AML_UART_CONTROL_CLRERR);
    writel(temp, ctrl_reg);

// configure baud rate and clock
    writel(0x01800045, reg5);
    
    // enable rx and tx
    temp |= AML_UART_CONTROL_TXEN | AML_UART_CONTROL_RXEN;
    // XMITLEN zero for 8 bits, PAREN zero for no parity, STOPLEN zero for 1 stop bit
    temp &= ~(AML_UART_CONTROL_XMITLEN_MASK | AML_UART_CONTROL_PAREN | AML_UART_CONTROL_STOPLEN_MASK);
    temp |= AML_UART_CONTROL_INVRTS | AML_UART_CONTROL_TXINTEN | AML_UART_CONTROL_RXINTEN | AML_UART_CONTROL_TWOWIRE;
    writel(temp, ctrl_reg);

    // Set to interrupt every 1 tx and rx byte
    temp = readl(irq_ctrl_reg);
    temp &= 0xffff0000;
    temp |= (1 << 8) | ( 1 );
    writel(temp, irq_ctrl_reg);

    while (1) {
        uint64_t slots;
        zx_status_t result = zx_interrupt_wait(port->irq_handle, &slots);
        if (result != ZX_OK) {
            zxlogf(ERROR, "aml_uart_irq_thread: zx_interrupt_wait got %d\n", result);
            break;
        }
        
        read_state(port);
    }
printf("aml_uart_irq_thread done\n");

    return 0;
}

static zx_status_t aml_uart_bind(void* ctx, zx_device_t* parent) {
    zx_status_t status;

    aml_uart_t* uart = calloc(1, sizeof(aml_uart_t));
    if (!uart) {
        return ZX_ERR_NO_MEMORY;
    }

    platform_device_protocol_t pdev;
    if ((status = device_get_protocol(parent, ZX_PROTOCOL_PLATFORM_DEV, &pdev)) != ZX_OK) {
        zxlogf(ERROR, "aml_uart_bind: ZX_PROTOCOL_PLATFORM_DEV not available\n");
        goto fail;
    }

    platform_bus_protocol_t pbus;
    if ((status = device_get_protocol(parent, ZX_PROTOCOL_PLATFORM_BUS, &pbus)) != ZX_OK) {
        zxlogf(ERROR, "aml_uart_bind: ZX_PROTOCOL_PLATFORM_BUS not available\n");
        goto fail;
    }

    pdev_device_info_t info;
    status = pdev_get_device_info(&pdev, &info);
    if (status != ZX_OK) {
        zxlogf(ERROR, "aml_uart_bind: pdev_get_device_info failed\n");
        goto fail;
    }
    if (info.mmio_count != info.irq_count) {
         zxlogf(ERROR, "aml_uart_bind: mmio_count %u does not match irq_count %u\n",
               info.mmio_count, info.irq_count);
        status = ZX_ERR_INVALID_ARGS;
        goto fail;
    }

    unsigned port_count = info.mmio_count;
    aml_uart_port_t* ports = calloc(port_count, sizeof(aml_uart_port_t));
    if (!ports) {
        status = ZX_ERR_NO_MEMORY;
        goto fail;
    }
    uart->ports = ports;
    uart->port_count = port_count;

    for (unsigned i = 0; i < port_count; i++) {
        aml_uart_port_t* port = &ports[i];
        mtx_init(&port->lock, mtx_plain);
        port->port_num = i;

        status = pdev_map_mmio_buffer(&pdev, i, ZX_CACHE_POLICY_UNCACHED_DEVICE, &port->mmio);
        if (status != ZX_OK) {
            zxlogf(ERROR, "aml_uart_bind: pdev_map_mmio_buffer failed %d\n", status);
            goto fail;
        }
        status = pdev_map_interrupt(&pdev, i, &port->irq_handle);
        if (status != ZX_OK) {
            zxlogf(ERROR, "aml_uart_bind: pdev_map_interrupt failed %d\n", status);
            goto fail;
        }
        int rc = thrd_create_with_name(&port->irq_thread, aml_uart_irq_thread, port,
                                       "aml_uart_irq_thread");
         if (rc != ZX_OK) {
            goto fail;
        }
   }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "aml-uart",
        .ctx = uart,
        .ops = &uart_device_proto,
        .flags = DEVICE_ADD_NON_BINDABLE,
    };

    status = device_add(parent, &args, &uart->zxdev);
    if (status != ZX_OK) {
        zxlogf(ERROR, "aml_uart_bind: device_add failed\n");
        goto fail;
    }

    uart->serial.ops = &aml_serial_ops;
    uart->serial.ctx = uart;
    pbus_set_protocol(&pbus, ZX_PROTOCOL_SERIAL_DRIVER, &uart->serial);

    return ZX_OK;

fail:
    aml_uart_release(uart);
    return status;
}

static zx_driver_ops_t aml_uart_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = aml_uart_bind,
};

ZIRCON_DRIVER_BEGIN(aml_uart, aml_uart_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PLATFORM_DEV),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_AMLOGIC),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_AMLOGIC_UART),
ZIRCON_DRIVER_END(aml_uart)
