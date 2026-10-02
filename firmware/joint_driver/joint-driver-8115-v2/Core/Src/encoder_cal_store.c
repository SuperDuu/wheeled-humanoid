#include "encoder_cal_store.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <string.h>

#define ENCODER_CAL_FLASH_ADDRESS 0x0807F800UL
#define ENCODER_CAL_MAGIC         0x314C4143UL /* "CAL1" */
#define ENCODER_CAL_VERSION       2U

typedef struct __attribute__((packed, aligned(8))) {
    uint32_t magic;
    uint16_t version;
    uint16_t lut_size;
    int16_t  lut[AS5048A_LUT_SIZE];
    float    zero_electric_angle;  /* Persisted electrical zero offset (radians) */
    int8_t   encoder_direction;    /* +1 or -1 */
    uint8_t  aligned;              /* 1 = valid alignment stored */
    uint16_t reserved16;
    uint32_t checksum;
    uint32_t reserved;
} encoder_cal_record_t;

_Static_assert((sizeof(encoder_cal_record_t) % 8U) == 0U,
               "Flash record must contain complete double words");

static uint32_t EncoderCalStore_Checksum(const encoder_cal_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    const size_t length = offsetof(encoder_cal_record_t, checksum);
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < length; i++) {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

static bool EncoderCalStore_IsValid(const encoder_cal_record_t *record)
{
    if (record->magic != ENCODER_CAL_MAGIC ||
        record->version != ENCODER_CAL_VERSION ||
        record->lut_size != AS5048A_LUT_SIZE ||
        record->checksum != EncoderCalStore_Checksum(record)) {
        return false;
    }

    for (size_t i = 0; i < AS5048A_LUT_SIZE; i++) {
        if (record->lut[i] < -256 || record->lut[i] > 256) {
            return false;
        }
    }
    return true;
}

static bool EncoderCalStore_WriteFlash(const encoder_cal_record_t *record)
{
    uint32_t bank = FLASH_BANK_1;
    uint32_t page = 0U;
#if defined(FLASH_OPTR_DBANK)
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U) {
        if (ENCODER_CAL_FLASH_ADDRESS >= (FLASH_BASE + FLASH_BANK_SIZE)) {
            bank = FLASH_BANK_2;
            page = (ENCODER_CAL_FLASH_ADDRESS - FLASH_BASE - FLASH_BANK_SIZE) /
                   FLASH_PAGE_SIZE;
        } else {
            page = (ENCODER_CAL_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
        }
    } else {
        page = (ENCODER_CAL_FLASH_ADDRESS - FLASH_BASE) /
               FLASH_PAGE_SIZE_128_BITS;
    }
#else
    page = (ENCODER_CAL_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
#endif

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = bank;
    erase.Page = page;
    erase.NbPages = 1U;

    uint32_t page_error = 0U;
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status == HAL_OK) {
        status = HAL_FLASHEx_Erase(&erase, &page_error);
    }

    const uint8_t *bytes = (const uint8_t *)record;
    for (size_t offset = 0; status == HAL_OK && offset < sizeof(encoder_cal_record_t); offset += 8U) {
        uint64_t value = 0U;
        memcpy(&value, &bytes[offset], sizeof(value));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   ENCODER_CAL_FLASH_ADDRESS + offset,
                                   value);
    }
    HAL_FLASH_Lock();

    if (status != HAL_OK) return false;

    encoder_cal_record_t verify;
    memcpy(&verify, (const void *)ENCODER_CAL_FLASH_ADDRESS, sizeof(verify));
    return EncoderCalStore_IsValid(&verify);
}

bool EncoderCalStore_Load(AS5048A_t *encoder, float *zero_electric_angle, int8_t *encoder_dir, bool *is_aligned)
{
    if (encoder == NULL) return false;

    encoder_cal_record_t record;
    memcpy(&record, (const void *)ENCODER_CAL_FLASH_ADDRESS, sizeof(record));
    if (!EncoderCalStore_IsValid(&record)) {
        encoder->use_lut = 0U;
        if (is_aligned != NULL) *is_aligned = false;
        return false;
    }

    memcpy(encoder->offset_lut, record.lut, sizeof(record.lut));
    encoder->use_lut = 1U;

    if (zero_electric_angle != NULL) {
        *zero_electric_angle = record.zero_electric_angle;
    }
    if (encoder_dir != NULL && (record.encoder_direction == 1 || record.encoder_direction == -1)) {
        *encoder_dir = record.encoder_direction;
    }
    if (is_aligned != NULL) {
        *is_aligned = (record.aligned != 0U);
    }
    return true;
}

bool EncoderCalStore_Save(const int16_t lut[AS5048A_LUT_SIZE], float zero_electric_angle, int8_t encoder_dir)
{
    encoder_cal_record_t record;
    memset(&record, 0, sizeof(record));
    record.magic = ENCODER_CAL_MAGIC;
    record.version = ENCODER_CAL_VERSION;
    record.lut_size = AS5048A_LUT_SIZE;
    if (lut != NULL) {
        memcpy(record.lut, lut, sizeof(record.lut));
    }
    record.zero_electric_angle = zero_electric_angle;
    record.encoder_direction = (encoder_dir == -1) ? -1 : 1;
    record.aligned = 1U;
    record.checksum = EncoderCalStore_Checksum(&record);

    return EncoderCalStore_WriteFlash(&record);
}

bool EncoderCalStore_SaveAlignment(float zero_electric_angle, int8_t encoder_dir)
{
    encoder_cal_record_t record;
    memcpy(&record, (const void *)ENCODER_CAL_FLASH_ADDRESS, sizeof(record));
    if (record.magic != ENCODER_CAL_MAGIC || record.version != ENCODER_CAL_VERSION) {
        memset(&record, 0, sizeof(record));
        record.magic = ENCODER_CAL_MAGIC;
        record.version = ENCODER_CAL_VERSION;
        record.lut_size = AS5048A_LUT_SIZE;
    }
    record.zero_electric_angle = zero_electric_angle;
    record.encoder_direction = (encoder_dir == -1) ? -1 : 1;
    record.aligned = 1U;
    record.checksum = EncoderCalStore_Checksum(&record);

    return EncoderCalStore_WriteFlash(&record);
}
