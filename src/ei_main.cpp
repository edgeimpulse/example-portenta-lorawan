#include "ei_main.h"
#include "ei_device_portenta.h"
#include "porting/ei_classifier_porting.h"
#include "ei_portenta_fs_commands.h"
#include "ei_microphone.h"
#include "ei_config_types.h"
#include "at_cmd_repl_mbed.h"
#include "ei_run_impulse.h"
#include "sensors/ei_camera.h"
#include <MKRWAN.h>

LoRaModem modem;

mbed::DigitalOut led(LED1);


EventQueue main_application_queue;
static unsigned char repl_stack[8 * 1024];
static AtCmdRepl repl(&main_application_queue, ei_get_serial(), sizeof(repl_stack), repl_stack, 5);

static String appEui = "70B3D57ED003BFCA";
static String appKey = "1F537E0DDCFCFF43443004F3A88C4294";

void fill_memory() {
    size_t size = 8 * 1024;
    size_t allocated = 0;
    while (1) {
        void *ptr = ei_malloc(size);
        if (!ptr) {
            if (size == 1) break;
            size /= 2;
        }
        else {
            allocated += size;
        }
    }
    ei_printf("Allocated: %u bytes\n", allocated);
}


#ifdef __MBED__
static bool ei_portenta_fs_read_buffer(size_t begin, size_t length, mbed::Callback<void(uint8_t*, size_t)> data_fn) {
#else
static bool ei_portenta_fs_read_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t)) {
#endif

    size_t pos = begin;
    size_t bytes_left = length;

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            return true;
        }

        int r = ei_portenta_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            return false;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    return true;

}

char last_event[32] = { 0 };
char last_message_str[32] = { 0 };
bool is_event_changed = false;

void event_changed(const char* event) {
    ei_printf("\tevent changed: %s\n", event);
    memcpy(last_event, event, 31);
    is_event_changed = true;
}

void run_nn_with_cb() {
    run_nn_normal(&event_changed);
}
void run_nn_debug_with_cb() {
    run_nn_debug(&event_changed);
}

void lorawan_main() {
    ei_printf("Hello from the LoRaWAN thread...\n");
    ThisThread::sleep_for(5000);

    if (!modem.begin(EU868)) {
        ei_printf("Failed to initialize modem\n");
    }

    ei_printf("[LRWN] Device EUI is: %s\n", modem.deviceEUI().c_str());

    bool connected = false;
    while (connected == false) {
        ei_printf("[LRWN] Joining network...\r\n");
        connected = modem.joinOTAA(appEui, appKey);
        if (connected) {
            ei_printf("[LRWN] Joined network!\r\n", connected);
        }
        else {
            ei_printf("[LRWN] Joining network failed, retrying in 10 seconds...\r\n", connected);
        }
        ThisThread::sleep_for(10000);
    }

    uint64_t last_message_sent = 0;

    while (1) {
        // event should be changed and last message should be >7000ms.
        if (is_event_changed && ei_read_timer_ms() - last_message_sent >= 7000) {
            // don't stream uncertain events
            if (strcmp(last_event, "uncertain") == 0 || strcmp(last_event, last_message_str) == 0) {
                ThisThread::sleep_for(1000);
                continue;
            }

            ei_printf("[LRWN] Sending event: '%s'\n", last_event);
            modem.dataRate(5);
            modem.setPort(3);
            modem.beginPacket();
            modem.print(last_event);
            int err = modem.endPacket(false);
            ei_printf("[LRWN] Sending event: '%s' returned %d\n", last_event, err);

            memcpy(last_message_str, last_event, 31);
            last_message_sent = ei_read_timer_ms();
            is_event_changed = false;
        }
        ThisThread::sleep_for(1000);
    }
}

void run_nn_at() {
    ei_at_cmd_handle("AT+RUNIMPULSE");
}

void ei_main() {
    ei_sleep(1000);


    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    ei_portenta_fs_init();
    ei_camera_init();

    // intialize configuration
    ei_config_ctx_t config_ctx = { 0 };
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function();
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();

    config_ctx.load_config = &ei_portenta_fs_load_config;
    config_ctx.save_config = &ei_portenta_fs_save_config;
    config_ctx.list_files = NULL;
    config_ctx.read_buffer = &ei_portenta_fs_read_buffer;
    config_ctx.take_snapshot = &ei_camera_take_snapshot_encode_and_output;
    config_ctx.start_snapshot_stream = &ei_camera_start_snapshot_stream_encode_and_output;

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\r\n", cr);
    }
    else {
        ei_printf("Loaded configuration\r\n");
    }

    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_with_cb);
    ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse", run_nn_debug_with_cb);
    // ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse continuously", run_nn_continuous_normal);
    ei_at_cmd_register("FILLMEMORY", "Fill memory", fill_memory);

    main_application_queue.call_in(2000, &run_nn_at);

    Thread lorawan_thread;
    lorawan_thread.start(&lorawan_main);

    EiDevice.set_state(eiStateIdle);

    repl.start_repl();
    main_application_queue.dispatch_forever();

    // while(1) {
    //     delay(500);
    //     ei_printf("portenta loop %lu\r\n", (uint32_t)ei_read_timer_ms());

    // }
}
