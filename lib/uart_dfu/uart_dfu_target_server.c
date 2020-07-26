#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <sys/atomic.h>
#include <uart_dfu.h>
#include <dfu/dfu_target.h>
#include <uart_dfu_target_server.h>


/* TODO: find a way to not do dfu_target calls from the uart_dfu workqueue context. */

/*****************************************************************************
 * Static variables 
 *****************************************************************************/

LOG_MODULE_REGISTER(uart_dfu_target_server, CONFIG_UART_DFU_TARGET_SERVER_LOG_LEVEL);


/*****************************************************************************
 * Static functions 
 *****************************************************************************/

static void dfu_target_callback(enum dfu_target_evt_id evt_id)
{
    switch (evt_id)
    {
        case DFU_TARGET_EVT_TIMEOUT:
        {
            /* TODO */
            break;
        }
        case DFU_TARGET_EVT_ERASE_DONE:
        {
            /* TODO */
            break;
        }
        default:
        {
            break;
        }
    }
}

static int dfu_server_init_callback(size_t file_size, void * context)
{
    struct uart_dfu_target_server * server;

    server = (struct uart_dfu_target_server *) context;

    if (atomic_cas(&server->file_size, 0, (atomic_val_t) file_size))
    {
        LOG_INF("Initialized(file_size=%u)", file_size);
        return 0;
    }
    else
    {
        LOG_ERR("Initialize failed: busy.");
        return -EBUSY;
    }
}

static int dfu_server_write_callback(const u8_t * const fragment_buf,
                                     size_t fragment_size,
                                     void * context)
{
    /* Init, write */
    int err;
    struct uart_dfu_target_server * server;

    server = (struct uart_dfu_target_server *) context;

    LOG_INF("Write(fragment_size=%u)", fragment_size);

    if (atomic_cas(&server->initialized, 0, 1))
    {
        /* First fragment, initialize. */
        int img_type;        
        
        img_type = dfu_target_img_type(fragment_buf, fragment_size);
        if (img_type < 0)
        {
            LOG_INF("dfu_target_img_type result: %d", img_type);
            return img_type;
        }

        err = dfu_target_init(img_type, server->file_size, dfu_target_callback);
        if (err < 0)
        {
            LOG_INF("dfu_target_init result: %d", err);
            return err;
        }
    }

    err = dfu_target_write(fragment_buf, fragment_size);

    LOG_INF("dfu_target_write result: %d", err);

    return err;
}

static int dfu_server_offset_callback(size_t * offset, void * context)
{
    int err;

    ARG_UNUSED(context);

    LOG_INF("Offset()");

    err = dfu_target_offset_get(offset);
    if (err == -EACCES)
    {
        /* The target is not yet initialized,
           so we just return an offset of 0. */
        *offset = 0;
        return 0;
    }
    else
    {
        return err;
    }
}

static int dfu_server_done_callback(bool successful, void * context)
{
    int err;
    struct uart_dfu_target_server * server;

    server = (struct uart_dfu_target_server *) context;

    LOG_INF("Done(successful=%u)", successful);

    err = dfu_target_done(successful);

    (void) atomic_set(&server->file_size, 0);
    (void) atomic_set(&server->initialized, 0); 

    return err;
}

static void dfu_server_error_callback(int error, void * context)
{
    LOG_ERR("Error(error=%d)", error);
    /* TODO: dfu_target_reset or similar? */
}


/*****************************************************************************
 * API functions 
 *****************************************************************************/

int uart_dfu_target_server_init(struct uart_dfu_target_server * server,
                                size_t inst_idx)
{
    int err;
    struct uart_dfu_server_callbacks callbacks;

    if (server == NULL)
    {
        return -EINVAL;
    }

    (void) atomic_set(&server->file_size, 0);
    (void) atomic_set(&server->initialized, 0);
    server->inst_idx = inst_idx;

    err = uart_dfu_server_init(&server->server,
                               server->fragment_buffer,
                               CONFIG_UART_DFU_TARGET_SERVER_MAX_FRAGMENT_SIZE);
    if (err != 0)
    {
        return err;
    }

    callbacks.init_callback = dfu_server_init_callback;
    callbacks.write_callback = dfu_server_write_callback;
    callbacks.offset_callback = dfu_server_offset_callback;
    callbacks.done_callback = dfu_server_done_callback;
    callbacks.error_callback = dfu_server_error_callback;

    err = uart_dfu_server_set(inst_idx,
                              &server->server,
                              &callbacks,
                              server);
    return err;
}

int uart_dfu_target_server_enable(struct uart_dfu_target_server * server)
{
    int err;

    if (server == NULL)
    {
        return -EINVAL;
    }

    err = uart_dfu_server_enable(server->inst_idx);

    return err;
}

int uart_dfu_target_server_disable(struct uart_dfu_target_server * server)
{
    int err;

    if (server == NULL)
    {
        return -EINVAL;
    }

    err = uart_dfu_server_disable(server->inst_idx);

    return err;
}