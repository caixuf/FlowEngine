/**
 * proto_support.c — Protobuf 适配层实现
 *
 * 编译: gcc -DFLOWENGINE_USE_PROTOBUF -lprotobuf-c
 */

#include "proto_support.h"

#ifdef FLOWENGINE_USE_PROTOBUF

#include "serializer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── Protobuf ↔ Message ────────────────────────────────── */

int msg_init_from_pb(Message* msg, const char* topic, const char* sender,
                     const ProtobufCMessageDescriptor* descriptor,
                     const ProtobufCMessage* pb_msg) {
    if (!msg || !descriptor || !pb_msg) return -1;

    /* Pack protobuf to binary */
    size_t packed_size = protobuf_c_message_get_packed_size(pb_msg);
    if (packed_size > MSG_BUS_MAX_DATA_SIZE) return -1;

    uint8_t* buf = (uint8_t*)malloc(packed_size);
    if (!buf) return -1;

    protobuf_c_message_pack(pb_msg, buf);
    /* protobuf_c_message_pack returns the size, actually:
       size_t actual = protobuf_c_message_pack(pb_msg, buf); */

    /* Build typed Message */
    uint32_t type_id = fnv1a_hash((const uint8_t*)descriptor->name,
                                   strlen(descriptor->name));
    msg_init_typed(msg, topic, sender, type_id, 1, buf, packed_size);

    free(buf);
    return 0;
}

ProtobufCMessage* msg_parse_to_pb(const Message* msg,
                                  const ProtobufCMessageDescriptor* descriptor) {
    if (!msg || !descriptor || msg->data_size == 0) return NULL;

    /* Type check */
    uint32_t expected_id = fnv1a_hash((const uint8_t*)descriptor->name,
                                       strlen(descriptor->name));
    if (msg->type_id != 0 && msg->type_id != expected_id) {
        fprintf(stderr, "[proto] type_id mismatch: msg=0x%08x expected=0x%08x (%s)\n",
                msg->type_id, expected_id, descriptor->name);
        return NULL;
    }

    /* Unpack */
    return protobuf_c_message_unpack(descriptor, NULL, msg->data_size, msg->data);
}

bool msg_validate_pb_type(const Message* msg,
                          const ProtobufCMessageDescriptor* descriptor) {
    if (!msg || !descriptor) return false;
    uint32_t expected_id = fnv1a_hash((const uint8_t*)descriptor->name,
                                       strlen(descriptor->name));
    return (msg->type_id == 0 || msg->type_id == expected_id);
}

/* ── Type Registration ─────────────────────────────────── */

void proto_register_type(const ProtobufCMessageDescriptor* descriptor) {
    if (!descriptor) return;

    uint32_t type_id = fnv1a_hash((const uint8_t*)descriptor->name,
                                   strlen(descriptor->name));

    TypeRegistryEntry entry;
    memset(&entry, 0, sizeof(entry));
    entry.type_id        = type_id;
    entry.schema_version = 1;
    entry.struct_size    = 0;  /* protobuf uses dynamic size */
    snprintf((char*)entry.type_name, SERIALIZER_TYPE_NAME_LEN,
             "pb:%s", descriptor->name);

    /* Protobuf-c handles serialization via pack/unpack, so we don't
     * register custom serialize/deserialize functions. */
    entry.serialize   = NULL;
    entry.deserialize = NULL;
    entry.endian_swap = NULL;

    serializer_register_type(&entry);
}

void proto_register_types(const ProtobufCMessageDescriptor** descriptors) {
    if (!descriptors) return;
    for (int i = 0; descriptors[i] != NULL; i++) {
        proto_register_type(descriptors[i]);
    }
}

uint32_t proto_get_type_id(const ProtobufCMessageDescriptor* descriptor) {
    if (!descriptor) return 0;
    return fnv1a_hash((const uint8_t*)descriptor->name,
                       strlen(descriptor->name));
}

#endif /* FLOWENGINE_USE_PROTOBUF */
