# zx_thread_write_state

## NAME

thread_write_state - Write one aspect of thread state.

## SYNOPSIS

```
#include <zircon/syscalls.h>
#include <zircon/syscalls/debug.h>

zx_status_t zx_thread_write_state(
    zx_handle_t handle,
    uint32_t kind,
    const void* buffer,
    size_t len);
```

## DESCRIPTION

**thread_write_state**() writes one aspect of state of the thread. The thread
state may only be written when the thread is halted for an exception.

The thread state is highly processor specific. See the structures in
zircon/syscalls/debug.h for the contents of the structures on each platform.

## STATES

See [thread_read_state](thread_read_state.md) for the list of available states
and their corresponding values.

## RETURN VALUE

**thread_write_state**() returns **ZX_OK** on success.
In the event of failure, a negative error value is returned.

## ERRORS

**ZX_ERR_BAD_HANDLE**  *handle* is not a valid handle.

**ZX_ERR_WRONG_TYPE**  *handle* is not that of a thread.

**ZX_ERR_ACCESS_DENIED**  *handle* lacks *ZX_RIGHT_WRITE*.

**ZX_ERR_INVALID_ARGS**  *kind* is not valid, *buffer* is an invalid pointer,
or *len* doesn't match the size of the structure expected for *kind*.

**ZX_ERR_NO_MEMORY**  Temporary out of memory failure.

**ZX_ERR_BAD_STATE**  The thread is not stopped at a point where state
is available. The thread state may only be read when the thread is stopped due
to an exception.

## SEE ALSO

[thread_read_state](thread_read_state.md).
