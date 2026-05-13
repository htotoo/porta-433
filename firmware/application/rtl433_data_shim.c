/** @file
 *  Arena-based replacement for rtl_433/src/data.c
 *
 *  All dynamic allocation (malloc/calloc/strdup/free) replaced with a
 *  static bump allocator.  data_free() resets the arena.  No heap
 *  operations happen in the decode path, so there is nothing to corrupt.
 *
 *  Copyright (C) 2015 Erkki Seppälä – original logic
 *  Arena adaptation for PortaPack M0 embedded target
 */

#include "data.h"
#include "abuf.h"

#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#define UNUSED(x) (void)(x)

/* ── Static arena ──────────────────────────────────────────────────────
 *
 * One arena per data_make/data_free cycle.  Sized for the most complex
 * decoder output we expect (Acurite-Atlas ~14 fields: ~1.3 KB).
 * 4 KB gives ample headroom.
 *
 * Every allocation is 8-byte aligned so data_value_t (containing a
 * double) is always naturally aligned on Cortex-M0 (which faults on
 * unaligned accesses).
 * ──────────────────────────────────────────────────────────────────── */

#define RTL433_ARENA_SIZE  4096u

static uint8_t  s_arena[RTL433_ARENA_SIZE];
static size_t   s_arena_pos = 0;

static void* arena_alloc(size_t sz)
{
    size_t aligned = (sz + 7u) & ~(size_t)7u;
    if (s_arena_pos + aligned > RTL433_ARENA_SIZE)
        return NULL;
    void* p = s_arena + s_arena_pos;
    memset(p, 0, aligned);
    s_arena_pos += aligned;
    return p;
}

static char* arena_strdup(const char* s)
{
    if (!s) return NULL;
    size_t len = strlen(s) + 1u;
    char* dst = (char*)arena_alloc(len);
    if (dst) memcpy(dst, s, len);
    return dst;
}

static void arena_reset(void)
{
    s_arena_pos = 0;
}

/* ── dmt table ──────────────────────────────────────────────────────── */

typedef void* (*array_elementwise_import_fn)(void*);
typedef void  (*array_element_release_fn)(void*);
typedef void  (*value_release_fn)(void*);

typedef struct {
    int    array_element_size;
    bool   array_is_boxed;
    array_elementwise_import_fn array_elementwise_import;
    array_element_release_fn    array_element_release;
    value_release_fn            value_release;
} data_meta_type_t;

static void* arena_strdup_import(void* s) { return arena_strdup((const char*)s); }

static data_meta_type_t dmt[DATA_COUNT] = {
    /* DATA_DATA   */ { (int)sizeof(data_t*),      true,  NULL,                 NULL, NULL },
    /* DATA_INT    */ { (int)sizeof(int),           false, NULL,                 NULL, NULL },
    /* DATA_DOUBLE */ { (int)sizeof(double),        false, NULL,                 NULL, NULL },
    /* DATA_STRING */ { (int)sizeof(char*),         true,  arena_strdup_import,  NULL, NULL },
    /* DATA_ARRAY  */ { (int)sizeof(data_array_t*), true,  NULL,                 NULL, NULL },
};

/* ── data_array ─────────────────────────────────────────────────────── */

R_API data_array_t* data_array(int num_values, data_type_t type, void const* values)
{
    if (num_values < 0) return NULL;

    data_array_t* array = (data_array_t*)arena_alloc(sizeof(data_array_t));
    if (!array) return NULL;

    int element_size = dmt[type].array_element_size;
    if (num_values > 0) {
        array->values = arena_alloc((size_t)num_values * (size_t)element_size);
        if (!array->values) return NULL;

        if (dmt[type].array_elementwise_import) {
            for (int i = 0; i < num_values; ++i) {
                void* src_elem = *(void**)((char*)values + element_size * i);
                void* copy = dmt[type].array_elementwise_import(src_elem);
                *((void**)((char*)array->values + element_size * i)) = copy;
            }
        } else {
            memcpy(array->values, values, (size_t)num_values * (size_t)element_size);
        }
    }

    array->num_values = num_values;
    array->type = type;
    return array;
}

/* ── vdata_make ─────────────────────────────────────────────────────── */

static data_t* vdata_make(data_t* first, const char* key, const char* pretty_key, va_list ap)
{
    data_type_t type;
    data_t* prev = first;
    while (prev && prev->next)
        prev = prev->next;

    char* format = NULL;
    int   skip   = 0;

    type = (data_type_t)va_arg(ap, int);

    do {
        data_t*      current;
        data_value_t value;
        memset(&value, 0, sizeof(value));

        switch (type) {
            case DATA_COND:
                skip |= !va_arg(ap, int);
                type = (data_type_t)va_arg(ap, int);
                continue;

            case DATA_FORMAT:
                format = va_arg(ap, char*);
                if (format) format = arena_strdup(format);
                type = (data_type_t)va_arg(ap, int);
                continue;

            case DATA_COUNT:
                goto alloc_error;

            case DATA_DATA:
                value.v_ptr = va_arg(ap, data_t*);
                break;

            case DATA_INT:
                value.v_int = va_arg(ap, int);
                break;

            case DATA_DOUBLE:
                value.v_dbl = va_arg(ap, double);
                break;

            case DATA_STRING:
                value.v_ptr = arena_strdup(va_arg(ap, char const*));
                break;

            case DATA_ARRAY:
                value.v_ptr = va_arg(ap, data_array_t*);
                break;

            default:
                goto alloc_error;
        }

        if (skip) {
            /* arena allocation – no individual releases needed */
            format = NULL;
            skip   = 0;
        } else {
            current = (data_t*)arena_alloc(sizeof(data_t));
            if (!current) goto alloc_error;

            current->type   = type;
            current->format = format;
            format = NULL;
            current->value  = value;
            current->next   = NULL;

            if (prev) prev->next = current;
            prev = current;
            if (!first) first = current;

            current->key = arena_strdup(key);
            if (!current->key) goto alloc_error;

            current->pretty_key = arena_strdup(pretty_key ? pretty_key : key);
            if (!current->pretty_key) goto alloc_error;
        }

        key = va_arg(ap, const char*);
        if (key) {
            pretty_key = va_arg(ap, const char*);
            type = (data_type_t)va_arg(ap, int);
        }
    } while (key);

    return first;

alloc_error:
    arena_reset();
    return NULL;
}

/* ── Public API ─────────────────────────────────────────────────────── */

R_API data_t* data_make(const char* key, const char* pretty_key, ...)
{
    va_list ap;
    va_start(ap, pretty_key);
    data_t* result = vdata_make(NULL, key, pretty_key, ap);
    va_end(ap);
    return result;
}

static data_t* data_append(data_t* first, const char* key, const char* pretty_key, ...)
{
    va_list ap;
    va_start(ap, pretty_key);
    data_t* result = vdata_make(first, key, pretty_key, ap);
    va_end(ap);
    return result;
}

R_API data_t* data_prepend(data_t* tail, data_t* head)
{
    if (!head) return tail;
    data_t* p = head;
    while (p->next) p = p->next;
    p->next = tail;
    return head;
}

R_API data_t* data_int(data_t* first, char const* key, char const* pretty_key,
                       char const* format, int val)
{
    return data_append(first, key, pretty_key, DATA_FORMAT, format, DATA_INT, val, NULL);
}

R_API data_t* data_dbl(data_t* first, char const* key, char const* pretty_key,
                       char const* format, double val)
{
    return data_append(first, key, pretty_key, DATA_FORMAT, format, DATA_DOUBLE, val, NULL);
}

R_API data_t* data_str(data_t* first, char const* key, char const* pretty_key,
                       char const* format, char const* val)
{
    return data_append(first, key, pretty_key, DATA_FORMAT, format, DATA_STRING, val, NULL);
}

R_API data_t* data_ary(data_t* first, char const* key, char const* pretty_key,
                       char const* format, data_array_t* val)
{
    return data_append(first, key, pretty_key, DATA_FORMAT, format, DATA_ARRAY, val, NULL);
}

R_API data_t* data_dat(data_t* first, char const* key, char const* pretty_key,
                       char const* format, data_t* val)
{
    return data_append(first, key, pretty_key, DATA_FORMAT, format, DATA_DATA, val, NULL);
}

R_API data_t* data_hex(data_t* first, char const* key, char const* pretty_key,
                       char const* format, uint8_t const* val, unsigned len, char* buf)
{
    if (!format || !*format) format = "%02x";
    char* p = buf;
    for (unsigned i = 0; i < len; ++i)
        p += sprintf(p, format, val[i]);
    *p = '\0';
    return data_append(first, key, pretty_key, DATA_FORMAT, NULL, DATA_STRING, buf, NULL);
}

/* ── data_array_free / data_retain / data_free ──────────────────────── */

R_API void data_array_free(data_array_t* array)
{
    UNUSED(array);
    /* arena memory – reset handled by data_free */
}

R_API data_t* data_retain(data_t* data)
{
    /* not used in single-output embedded path */
    return data;
}

R_API void data_free(data_t* data)
{
    UNUSED(data);
    /* Reset the arena – all nodes and strings are invalidated at once */
    arena_reset();
}

/* ── Output helpers ─────────────────────────────────────────────────── */

R_API void print_value(data_output_t* output, data_type_t type,
                       data_value_t value, char const* format)
{
    switch (type) {
        case DATA_COND:
        case DATA_FORMAT:
        case DATA_COUNT:
            assert(0);
            break;
        case DATA_DATA:
            output->print_data(output, (data_t*)value.v_ptr, format);
            break;
        case DATA_INT:
            output->print_int(output, value.v_int, format);
            break;
        case DATA_DOUBLE:
            output->print_double(output, value.v_dbl, format);
            break;
        case DATA_STRING:
            output->print_string(output, (char const*)value.v_ptr, format);
            break;
        case DATA_ARRAY:
            output->print_array(output, (data_array_t*)value.v_ptr, format);
            break;
    }
}

R_API void print_array_value(data_output_t* output, data_array_t* array,
                             char const* format, int idx)
{
    int element_size = dmt[array->type].array_element_size;
    data_value_t value;
    memset(&value, 0, sizeof(value));

    if (!dmt[array->type].array_is_boxed) {
        memcpy(&value, (char*)array->values + element_size * idx, (size_t)element_size);
        print_value(output, array->type, value, format);
    } else {
        value.v_ptr = *(void**)((char*)array->values + element_size * idx);
        print_value(output, array->type, value, format);
    }
}

/* ── data_print_jsons ───────────────────────────────────────────────── */

typedef struct {
    struct data_output output;
    abuf_t msg;
} data_print_jsons_t;

static void R_API_CALLCONV format_jsons_array(data_output_t* output,
                                               data_array_t* array,
                                               char const* format)
{
    data_print_jsons_t* jsons = (data_print_jsons_t*)output;
    abuf_cat(&jsons->msg, "[");
    for (int c = 0; c < array->num_values; ++c) {
        if (c) abuf_cat(&jsons->msg, ",");
        print_array_value(output, array, format, c);
    }
    abuf_cat(&jsons->msg, "]");
}

static void R_API_CALLCONV format_jsons_object(data_output_t* output,
                                                data_t* data,
                                                char const* format)
{
    UNUSED(format);
    data_print_jsons_t* jsons = (data_print_jsons_t*)output;
    bool separator = false;
    abuf_cat(&jsons->msg, "{");
    while (data) {
        if (separator) abuf_cat(&jsons->msg, ",");
        output->print_string(output, data->key, NULL);
        abuf_cat(&jsons->msg, ":");
        print_value(output, data->type, data->value, data->format);
        separator = true;
        data = data->next;
    }
    abuf_cat(&jsons->msg, "}");
}

static void R_API_CALLCONV format_jsons_string(data_output_t* output,
                                                const char* str,
                                                char const* format)
{
    UNUSED(format);
    data_print_jsons_t* jsons = (data_print_jsons_t*)output;
    char*  buf  = jsons->msg.tail;
    size_t size = jsons->msg.left;

    if (!str || size < 3) return;

    size_t str_len = strlen(str);
    if (str_len > 0 && str[0] == '{' && str[str_len - 1] == '}') {
        abuf_cat(&jsons->msg, str);
        return;
    }
    if (size < str_len + 3) return;

    *buf++ = '"'; size--;
    for (; *str && size >= 3; ++str) {
        if (*str == '\r') { *buf++ = '\\'; *buf++ = 'r'; size -= 2; continue; }
        if (*str == '\n') { *buf++ = '\\'; *buf++ = 'n'; size -= 2; continue; }
        if (*str == '\t') { *buf++ = '\\'; *buf++ = 't'; size -= 2; continue; }
        if (*str == '"' || *str == '\\') { *buf++ = '\\'; size--; }
        *buf++ = *str; size--;
    }
    if (size >= 2) { *buf++ = '"'; size--; }
    *buf = '\0';
    jsons->msg.tail = buf;
    jsons->msg.left = size;
}

static void R_API_CALLCONV format_jsons_double(data_output_t* output,
                                                double data,
                                                char const* format)
{
    UNUSED(format);
    data_print_jsons_t* jsons = (data_print_jsons_t*)output;
    if (data > 1e7 || data < 1e-4)
        abuf_printf(&jsons->msg, "%g", data);
    else {
        abuf_printf(&jsons->msg, "%.5f", data);
        while (jsons->msg.left > 0
               && *(jsons->msg.tail - 1) == '0'
               && *(jsons->msg.tail - 2) != '.') {
            jsons->msg.tail--;
            jsons->msg.left++;
            *jsons->msg.tail = '\0';
        }
    }
}

static void R_API_CALLCONV format_jsons_int(data_output_t* output,
                                             int data,
                                             char const* format)
{
    UNUSED(format);
    data_print_jsons_t* jsons = (data_print_jsons_t*)output;
    abuf_printf(&jsons->msg, "%d", data);
}

R_API size_t data_print_jsons(data_t* data, char* dst, size_t len)
{
    data_print_jsons_t jsons = {
        .output = {
            .print_data   = format_jsons_object,
            .print_array  = format_jsons_array,
            .print_string = format_jsons_string,
            .print_double = format_jsons_double,
            .print_int    = format_jsons_int,
        },
    };
    abuf_init(&jsons.msg, dst, len);
    format_jsons_object(&jsons.output, data, NULL);
    return len - jsons.msg.left;
}

/* ── data_output stubs ──────────────────────────────────────────────── */

R_API void data_output_print(data_output_t* output, data_t* data)
{
    if (!output) return;
    if (output->output_print)
        output->output_print(output, data);
    else
        output->print_data(output, data, NULL);
}

R_API void data_output_start(struct data_output* output,
                             char const* const* fields, int num_fields)
{
    if (!output || !output->output_start) return;
    output->output_start(output, fields, num_fields);
}

R_API void data_output_free(data_output_t* output)
{
    if (!output) return;
    output->output_free(output);
}
