#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <sys/atomic.h>
#include <uart_dfu.h>
#include <dfu/dfu_target.h>
#include <uart_dfu_target_server.h>

// TODO: Move to config
#define FRAGMENT_BUFFER_SIZE 4096

/*****************************************************************************
 * Static variables 
 *****************************************************************************/

static struct uart_dfu_server dfu_server;
static atomic_t dfu_file_size;
static u8_t fragment_buffer[FRAGMENT_BUFFER_SIZE];


/* FIXME: Configurable log level. */
LOG_MODULE_REGISTER(uart_dfu_target_server, LOG_LEVEL_DBG);


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

static int dfu_server_init_callback(size_t file_size)
{
    if (atomic_cas(&dfu_file_size, 0, (atomic_t) file_size))
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
                                     size_t fragment_size)
{
    /* Init, write */
    int err_code;
    size_t offset;

    LOG_INF("Write(fragment_size=%u)", fragment_size);

    err_code = dfu_target_offset_get(&offset);
    if ((err_code == 0 && offset == 0) ||
         err_code == -EACCES)
    {
        /* First fragment, initialize. */
        int img_type;        
        
        img_type = dfu_target_img_type(fragment_buf, fragment_size);
        if (img_type < 0)
        {
            return img_type;
        }

        err_code = dfu_target_init(img_type, dfu_file_size, dfu_target_callback);
        if (err_code < 0)
        {
            return err_code;
        }
    }

    err_code = dfu_target_write(fragment_buf, fragment_size);
    return err_code;
}

static int dfu_server_offset_callback(size_t * offset)
{
    int err_code;

    LOG_INF("Offset()");

    err_code = dfu_target_offset_get(offset);
    if (err_code == -EACCES)
    {
        /* The target is not yet initialized,
           so we just return an offset of 0. */
        *offset = 0;
        return 0;
    }
    else
    {
        return err_code;
    }
}

static int dfu_server_done_callback(bool successful)
{
    int err_code;

    LOG_INF("Done(successful=%u)", successful);

    err_code = dfu_target_done(successful);
    atomic_set(&dfu_file_size, 0);

    return err_code;
}

static void dfu_server_error_callback(int error)
{
    LOG_ERR("Error(error=%d)", error);
    /* TODO: dfu_target_reset or similar? */
}


/*****************************************************************************
 * API functions 
 *****************************************************************************/

int uart_dfu_target_server_init(void)
{
    int err_code;
    struct uart_dfu_server_callbacks callbacks;

    (void) atomic_set(&dfu_file_size, 0);
    
    callbacks.init_callback = dfu_server_init_callback;
    callbacks.write_callback = dfu_server_write_callback;
    callbacks.offset_callback = dfu_server_offset_callback;
    callbacks.done_callback = dfu_server_done_callback;
    callbacks.error_callback = dfu_server_error_callback;


    err_code = uart_dfu_server_set(&dfu_server,
                                   fragment_buffer,
                                   FRAGMENT_BUFFER_SIZE,
                                   &callbacks);
    return err_code;
}

int uart_dfu_target_server_enable(void)
{
    int err_code;

    err_code = uart_dfu_server_enable();

    return err_code;
}

int uart_dfu_target_server_disable(void)
{
    int err_code;

    err_code = uart_dfu_server_disable();

    return err_code;
}