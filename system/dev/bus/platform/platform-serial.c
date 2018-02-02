// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/protocol/serial.h>
#include <sync/completion.h>
#include <zircon/threads.h>
#include <stdlib.h>
#include <threads.h>

#include "platform-bus.h"

typedef struct serial_port {
    serial_driver_protocol_t serial;
    uint32_t port_num;
    zx_handle_t socket;
    thrd_t in_thread;
    thrd_t out_thread;
    completion_t readable;
    completion_t writeable;
    mtx_t lock;
} serial_port_t;

enum {
    WAIT_SOCKET,
    WAIT_EVENT,
};

#define UART_BUFFER_SIZE    1024

static int serial_in_thread(void* arg) {
    serial_port_t* port = arg;
    uint8_t buffer[UART_BUFFER_SIZE];

    while (1) {
// make sure we unblock this when shutting down
        completion_wait(&port->readable, ZX_TIME_INFINITE);
        size_t length;
        zx_status_t status = serial_driver_read(&port->serial, port->port_num, buffer,
                                                sizeof(buffer), &length);
        if (status == ZX_ERR_SHOULD_WAIT) {
            zxlogf(ERROR, "serial_in_thread: ZX_ERR_SHOULD_WAIT\n");
        } else if (status != ZX_OK) {
            zxlogf(ERROR, "serial_in_thread: serial_driver_read returned %d\n", status);
            break;
        }

        uint8_t* bufptr = buffer;
        while (length > 0) {
            size_t actual;
            zx_status_t status = zx_socket_write(port->socket, 0, bufptr, length, &actual);
            if (status == ZX_ERR_SHOULD_WAIT) {
                zx_signals_t observed;
                status = zx_object_wait_one(port->socket, ZX_SOCKET_WRITABLE | ZX_SOCKET_PEER_CLOSED, 
                                            ZX_TIME_INFINITE, &observed);
                if (status != ZX_OK) {
                    zxlogf(ERROR, "serial_in_thread: zx_object_wait_one returned %d\n", status);
                    break;
                }
                if (observed & ZX_SOCKET_PEER_CLOSED) {
// set a flag somewhere too? or close handle?
                    break;
                }
                if (observed & ZX_SOCKET_WRITABLE) {
                    continue;
                }
            } else if (status != ZX_OK) {
                zxlogf(ERROR, "serial_in_thread: zx_socket_read returned %d\n", status);
                break;
            }

            bufptr += actual;
            length -= actual;
        }
    }

    return 0;
}

static int serial_out_thread(void* arg) {
    serial_port_t* port = arg;
    uint8_t buffer[UART_BUFFER_SIZE];

    while (1) {
        zx_signals_t observed;
        zx_status_t status = zx_object_wait_one(port->socket,
                                                ZX_SOCKET_READABLE | ZX_SOCKET_PEER_CLOSED, 
                                                ZX_TIME_INFINITE, &observed);
        if (observed & ZX_SOCKET_READABLE) {
            size_t length;
            status = zx_socket_read(port->socket, 0, buffer, sizeof(buffer), &length);
            if (status != ZX_OK) {
                zxlogf(ERROR, "serial_out_thread: zx_socket_read returned %d\n", status);
                break;
            }

            uint8_t* bufptr = buffer;
            while (length > 0) {
// make sure we unblock this when shutting down
                completion_wait(&port->writeable, ZX_TIME_INFINITE);
                size_t actual;
                zx_status_t status = serial_driver_write(&port->serial, port->port_num, bufptr,
                                                         length, &actual);
                if (status == ZX_ERR_SHOULD_WAIT) {
                    continue;
                } else if (status != ZX_OK) {
                    zxlogf(ERROR, "serial_out_thread: serial_driver_write returned %d\n", status);
                    return -1;
                }
                bufptr += actual;
                length -= actual;
            }
        }
        if (observed & ZX_SOCKET_PEER_CLOSED) {
// set a flag somewhere too? or close handle?
            break;
        }
    }

    return 0;
}

static void platform_serial_state_cb(uint32_t port_num, uint32_t state, void* cookie) {
    serial_port_t* port = cookie;

    if (state & SERIAL_STATE_READABLE) {
        completion_signal(&port->readable);
    } else {
        completion_reset(&port->readable);
    }
    if (state & SERIAL_STATE_WRITABLE) {
        completion_signal(&port->writeable);
    } else {
        completion_reset(&port->writeable);
    }
}

zx_status_t platform_serial_init(platform_bus_t* bus, serial_driver_protocol_t* serial) {
    uint32_t port_count = serial_driver_get_port_count(serial);
    if (!port_count) {
        return ZX_ERR_INVALID_ARGS;
     }

    if (bus->serial_ports) {
        // already initialized
        return ZX_ERR_BAD_STATE;
    }

    serial_port_t* ports = calloc(port_count, sizeof(serial_port_t));
    if (!ports) {
        return ZX_ERR_NO_MEMORY;
    }

    for (uint32_t i = 0; i < port_count; i++) {
        serial_port_t* port = &ports[i];
        mtx_init(&port->lock, mtx_plain);
        memcpy(&port->serial, serial, sizeof(port->serial));
        port->port_num = i;
    }

    bus->serial_ports = ports;
    bus->serial_port_count = port_count;
    return ZX_OK;
}

void platform_serial_release(platform_bus_t* bus) {
    // TODO more cleanup
    free(bus->serial_ports);
}

zx_status_t platform_serial_config(platform_bus_t* bus, uint32_t port_num, uint32_t baud_rate,
                                   uint32_t flags) {
    if (port_num >= bus->serial_port_count) {
        return ZX_ERR_NOT_FOUND;
    }
    serial_port_t* port = &bus->serial_ports[port_num];

    mtx_lock(&port->lock);

    mtx_unlock(&port->lock);

    return ZX_OK;
}

zx_status_t platform_serial_open_socket(platform_bus_t* bus, uint32_t port_num,
                                        zx_handle_t* out_handle) {
    if (port_num >= bus->serial_port_count) {
        return ZX_ERR_NOT_FOUND;
    }
    serial_port_t* port = &bus->serial_ports[port_num];

    mtx_lock(&port->lock);
    if (port->socket != ZX_HANDLE_INVALID) {
        mtx_unlock(&port->lock);
        return ZX_ERR_ALREADY_BOUND;
    }

    zx_handle_t socket;
    zx_status_t status = zx_socket_create(ZX_SOCKET_STREAM, &port->socket, &socket);
    if (status != ZX_OK) {
        goto fail;
    }

    serial_driver_set_notify_callback(&bus->serial, port_num, platform_serial_state_cb, port);
// unregister on socket close?

    int thrd_rc = thrd_create_with_name(&port->in_thread, serial_in_thread, port,
                                        "serial_in_thread");
    if (thrd_rc != thrd_success) {
        status = thrd_status_to_zx_status(thrd_rc);
        goto fail;
    }
    thrd_rc = thrd_create_with_name(&port->out_thread, serial_out_thread, port,
                                   "serial_out_thread");
    if (thrd_rc != thrd_success) {
        status = thrd_status_to_zx_status(thrd_rc);
        goto fail;
    }

    *out_handle = socket;
    mtx_unlock(&port->lock);
    return ZX_OK;

fail:
// TODO join thread?
    zx_handle_close(port->socket);
    zx_handle_close(socket);
    port->socket = ZX_HANDLE_INVALID;

    mtx_unlock(&port->lock);
    return status;
}
