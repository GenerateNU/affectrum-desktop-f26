#ifndef PILL_SERVICE_H
#define PILL_SERVICE_H

#include <stdint.h>
#include <zephyr/toolchain.h>

// Wire format notes (share these with the phone app developer):
//   - every multi-byte field is little-endian
//   - timestamps are Unix seconds, UTC
//   - pill names are UTF-8, zero-padded, at most PILL_NAME_LEN - 1 bytes long
//   - pill ID 0 means "no pill" / "empty container"

#define PILL_CONTAINERS 4
#define PILL_NAME_LEN 30

// One pill in a dispense log entry
struct pill_map {
    uint16_t pill_id;
    uint8_t disp_cnt;
} __packed;

BUILD_ASSERT(sizeof(struct pill_map) == 2 + 1, "pill_map must be packed");

// One dispense event: when it happened and what came out of each container
struct log_entry {
    uint32_t timestamp;
    uint8_t num_pills;
    struct pill_map pills[PILL_CONTAINERS];
} __packed;

BUILD_ASSERT(sizeof(struct log_entry) == 4 + 1 + 3 * PILL_CONTAINERS, "log_entry must be packed");
BUILD_ASSERT(sizeof(struct log_entry) <= 20, "log_entry must fit in one default-size notification");

// Template for one container: which pill is loaded in it and how many to dispense per dose.
// The phone writes one of these at a time; reading returns all PILL_CONTAINERS of them.
struct pill_template {
    uint8_t container;          // 0 .. PILL_CONTAINERS - 1
    uint16_t pill_id;           // 0 = container empty
    uint8_t dose;               // pills to dispense per dose
    char name[PILL_NAME_LEN];   // UTF-8, zero-padded
} __packed;

BUILD_ASSERT(sizeof(struct pill_template) == 1 + 2 + 1 + PILL_NAME_LEN, "pill_template must be packed");

struct pill_container {
    uint8_t container_num;
    uint16_t curr_amt;
    uint16_t max_amt;
} __packed;

// Copies the template for one container into *out.
// Returns 0 on success, -EINVAL if container is out of range, -ENOENT if the container is empty.
int pill_service_get_template(uint8_t container, struct pill_template *out);

// Saves a dispense event as the latest log entry and notifies the phone if it has subscribed.
// Returns 0 on success (including when no phone is connected), or a negative error code.
int pill_service_log_dispense(const struct log_entry *entry);

#endif
