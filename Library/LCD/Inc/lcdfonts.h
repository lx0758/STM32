/* vim: set ai et ts=4 sw=4: */
#ifndef __LCDFONTS_H__
#define __LCDFONTS_H__

#include <stdint.h>

typedef struct {
  const uint8_t width;
  const uint8_t height;
  const uint16_t *data;
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif // __LCDFONTS_H__
