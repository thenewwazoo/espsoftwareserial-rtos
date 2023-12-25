#pragma once

inline uint32_t esp_get_cycle_count() __attribute__((always_inline));
inline uint32_t esp_get_cycle_count() {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a"(ccount));
  return ccount;
}

#if defined(F_CPU) || defined(CORE_MOCK)
#ifdef __cplusplus
constexpr
#else
inline
#endif
int esp_get_cpu_freq_mhz()
{
    return F_CPU / 1000000L;
}
#else
inline int esp_get_cpu_freq_mhz()
{
    uint8_t system_get_cpu_freq(void);
    return system_get_cpu_freq();
}
#endif

// from https://github.com/esp8266/Arduino/blob/d5eb265f78bff9deb7063d10030a02d021c8c66c/cores/esp8266/esp8266_peri.h
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
//GPIO (0-15) Control Registers
#define GPO    ESP8266_REG(0x300) //GPIO_OUT R/W (Output Level)
#define GPOS   ESP8266_REG(0x304) //GPIO_OUT_SET WO
#define GPOC   ESP8266_REG(0x308) //GPIO_OUT_CLR WO
//#define GPE    ESP8266_REG(0x30C) //GPIO_ENABLE R/W (Enable)
//#define GPES   ESP8266_REG(0x310) //GPIO_ENABLE_SET WO
//#define GPEC   ESP8266_REG(0x314) //GPIO_ENABLE_CLR WO
#define GPI    ESP8266_REG(0x318) //GPIO_IN RO (Read Input Level)
//#define GPIE   ESP8266_REG(0x31C) //GPIO_STATUS R/W (Interrupt Enable)
//#define GPIES  ESP8266_REG(0x320) //GPIO_STATUS_SET WO
//#define GPIEC  ESP8266_REG(0x324) //GPIO_STATUS_CLR WO
//GPIO 16 Control Registers
#define GP16O  ESP8266_REG(0x768)
//#define GP16E  ESP8266_REG(0x774)
#define GP16I  ESP8266_REG(0x78C)

// from https://github.com/esp8266/Arduino/blob/d5eb265f78bff9deb7063d10030a02d021c8c66c/cores/esp8266/Arduino.h#L202
#define _PORT_GPIO16    1
#define digitalPinToPort(pin)       (((pin)==16)?(_PORT_GPIO16):(0))
#define digitalPinToBitMask(pin)    (((pin)==16)?(1):(1UL << (pin)))
//#define portOutputRegister(port)    (((port)==_PORT_GPIO16)?((volatile uint32_t*) &GP16O):((volatile uint32_t*) &GPO))
#define portInputRegister(port)     (((port)==_PORT_GPIO16)?((volatile uint32_t*) &GP16I):((volatile uint32_t*) &GPI))
//#define portModeRegister(port)      (((port)==_PORT_GPIO16)?((volatile uint32_t*) &GP16E):((volatile uint32_t*) &GPE))

// from https://github.com/esp8266/Arduino/blob/d5eb265f78bff9deb7063d10030a02d021c8c66c/variants/generic/common.h#L39
#define EXTERNAL_NUM_INTERRUPTS 16
#define digitalPinToInterrupt(p)    (((p) < EXTERNAL_NUM_INTERRUPTS)? (p) : (-1))


// from https://github.com/esp8266/Arduino/blob/d5eb265f78bff9deb7063d10030a02d021c8c66c/tests/host/sys/pgmspace.h#L30
#define pgm_read_byte(addr) (*reinterpret_cast<const uint8_t*>(addr))
