// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/platform-defs.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/serial.h>

#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

typedef struct {
    zx_device_t* zxdev;
    serial_protocol_t serial;
    zx_handle_t socket;
    thrd_t thread;
    bool done;
} uart_test_t;

static void uart_test_release(void* ctx) {
    uart_test_t* test = ctx;

    test->done = true;
    thrd_join(test->thread, NULL);
    zx_handle_close(test->socket);
    free(test);
}

static zx_protocol_device_t uart_test_device_protocol = {
    .version = DEVICE_OPS_VERSION,
    .release = uart_test_release,
};

static int uart_test_thread(void *arg) {
    uart_test_t* test = arg;

    while (!test->done) {
        const char* hi_there = "Hi there!\n";
        zx_socket_write(test->socket, 0, hi_there, strlen(hi_there), NULL);
        sleep(1);
    }

    return 0;
}

static zx_status_t uart_test_bind(void* ctx, zx_device_t* parent) {
    serial_protocol_t serial;

    zx_status_t status = device_get_protocol(parent, ZX_PROTOCOL_SERIAL, &serial);
    if (status != ZX_OK) {
        zxlogf(ERROR, "uart_test_bind: get protocol ZX_PROTOCOL_SERIAL failed\n");
        return status;
    }

    uart_test_t* test = calloc(1, sizeof(uart_test_t));
    if (!test) {
        return ZX_ERR_NO_MEMORY;
    }

    status = serial_open_socket(&serial, 0, &test->socket);
     if (status != ZX_OK) {
        zxlogf(ERROR, "uart_test_bind: serial_open_socket failed: %d\n", status);
        free(test);
        return status;
    }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "uart-test",
        .ctx = test,
        .ops = &uart_test_device_protocol,
        .flags = DEVICE_ADD_NON_BINDABLE,
    };

    status = device_add(parent, &args, NULL);
    if (status != ZX_OK) {
        uart_test_release(test);
        return status;
    }

    thrd_create_with_name(&test->thread, uart_test_thread, test, "uart_test_thread");
    return ZX_OK;
}

static zx_driver_ops_t uart_test_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = uart_test_bind,
};

// clang-format off
ZIRCON_DRIVER_BEGIN(uart_test, uart_test_driver_ops, "zircon", "0.1", 2)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PLATFORM_DEV),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_UART_TEST),
ZIRCON_DRIVER_END(uart_test)
