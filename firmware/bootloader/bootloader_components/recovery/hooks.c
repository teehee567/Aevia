#include <string.h>
#include "bootloader_flash_priv.h"
#include "bootloader_utility.h"
#include "esp_rom_sys.h"
#include "soc/reset_reasons.h"

// Must match src/loader.rs and partitions.csv. Unknown/torn records select recovery.
#define STATE 0x9000
#define SECTOR 0x1000
#define APP_SIZE 0xf70000
#define RECOVERY 0x31434552u // REC1
#define START_APP 0x314e5552u // RUN1
#define APP 0x31505041u // APP1

void bootloader_hooks_include(void) {}

static bool set_boot(uint32_t marker, uint32_t size)
{
    uint32_t record[2] = {marker, size};
    uint32_t check[2];
    return bootloader_flash_erase_range(STATE, SECTOR) == ESP_OK
        && bootloader_flash_write(STATE + 4, &record[1], 4, false) == ESP_OK
        && bootloader_flash_write(STATE, &record[0], 4, false) == ESP_OK
        && bootloader_flash_read(STATE, check, sizeof(check), false) == ESP_OK
        && memcmp(record, check, sizeof(record)) == 0;
}

void bootloader_after_init(void)
{
    bootloader_state_t partitions = {0};
    if (!bootloader_utility_load_partition_table(&partitions)) {
        bootloader_reset();
    }

    uint32_t record[2] = {0};
    bool readable = bootloader_flash_read(STATE, record, sizeof(record), false) == ESP_OK;
    bool valid_size = record[1] >= 288 && record[1] <= APP_SIZE;
    unsigned reason = esp_rom_get_reset_reason(0);
    bool normal_boot = reason == RESET_REASON_CHIP_POWER_ON
        || reason == RESET_REASON_CORE_DEEP_SLEEP;
    bool run = readable && valid_size
        && (record[0] == START_APP || (record[0] == APP && normal_boot));

    if (run && record[0] == START_APP) {
        // Consume the launch request before loading any application code.
        run = set_boot(APP, record[1]);
    } else if (!run && readable && record[0] == APP) {
        // A fault reset must remain in recovery even after power is removed.
        set_boot(RECOVERY, valid_size ? record[1] : 0);
    }

    if (!run) {
        // Never try the application as a fallback when recovery was requested.
        partitions.app_count = 0;
    }
    // ESP-IDF validates the complete image and falls back from OTA-0 to factory.
    bootloader_utility_load_boot_image(&partitions, run ? 0 : FACTORY_INDEX);
}
