#include "ssd1306_simple.h"

#include <stdio.h>
#include <string.h>

#define SSD1306_ADDRESS (0x3cU << 1U)
#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)
#define SSD1306_TIMEOUT_MS 100U

static I2C_HandleTypeDef *display_i2c;
static uint8_t display_buffer[SSD1306_BUFFER_SIZE];

static bool WriteCommands(const uint8_t *commands, uint16_t count)
{
  uint8_t packet[32];

  if (display_i2c == NULL || commands == NULL ||
      count == 0U || count > (sizeof(packet) - 1U))
  {
    return false;
  }

  packet[0] = 0x00U;
  memcpy(&packet[1], commands, count);
  return HAL_I2C_Master_Transmit(display_i2c, SSD1306_ADDRESS, packet,
                                 (uint16_t)(count + 1U),
                                 SSD1306_TIMEOUT_MS) == HAL_OK;
}

static bool FlushDisplay(void)
{
  static const uint8_t address_commands[] = {
      0x21U, 0x00U, 0x7fU,
      0x22U, 0x00U, 0x07U,
  };

  if (!WriteCommands(address_commands, sizeof(address_commands)))
  {
    return false;
  }

  return HAL_I2C_Mem_Write(display_i2c, SSD1306_ADDRESS, 0x40U,
                           I2C_MEMADD_SIZE_8BIT, display_buffer,
                           sizeof(display_buffer),
                           SSD1306_TIMEOUT_MS) == HAL_OK;
}

static void SetPixel(uint8_t x, uint8_t y)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
  {
    return;
  }
  display_buffer[x + (uint16_t)(y / 8U) * SSD1306_WIDTH] |=
      (uint8_t)(1U << (y % 8U));
}

static void GetGlyph(char character, uint8_t glyph[5])
{
  static const uint8_t digits[10][5] = {
      {0x3eU, 0x51U, 0x49U, 0x45U, 0x3eU},
      {0x00U, 0x42U, 0x7fU, 0x40U, 0x00U},
      {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
      {0x21U, 0x41U, 0x45U, 0x4bU, 0x31U},
      {0x18U, 0x14U, 0x12U, 0x7fU, 0x10U},
      {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
      {0x3cU, 0x4aU, 0x49U, 0x49U, 0x30U},
      {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
      {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
      {0x06U, 0x49U, 0x49U, 0x29U, 0x1eU},
  };
  static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
  const uint8_t *source = blank;

  if (character >= 'a' && character <= 'z')
  {
    character = (char)(character - 'a' + 'A');
  }

  if (character >= '0' && character <= '9')
  {
    source = digits[(uint8_t)(character - '0')];
  }
  else
  {
    switch (character)
    {
      case 'A':
      {
        static const uint8_t data[5] = {0x7eU, 0x11U, 0x11U, 0x11U, 0x7eU};
        source = data;
        break;
      }
      case 'B':
      {
        static const uint8_t data[5] = {0x7fU, 0x49U, 0x49U, 0x49U, 0x36U};
        source = data;
        break;
      }
      case 'D':
      {
        static const uint8_t data[5] = {0x7fU, 0x41U, 0x41U, 0x22U, 0x1cU};
        source = data;
        break;
      }
      case 'E':
      {
        static const uint8_t data[5] = {0x7fU, 0x49U, 0x49U, 0x49U, 0x41U};
        source = data;
        break;
      }
      case 'F':
      {
        static const uint8_t data[5] = {0x7fU, 0x09U, 0x09U, 0x09U, 0x01U};
        source = data;
        break;
      }
      case 'G':
      {
        static const uint8_t data[5] = {0x3eU, 0x41U, 0x49U, 0x49U, 0x7aU};
        source = data;
        break;
      }
      case 'H':
      {
        static const uint8_t data[5] = {0x7fU, 0x08U, 0x08U, 0x08U, 0x7fU};
        source = data;
        break;
      }
      case 'I':
      {
        static const uint8_t data[5] = {0x00U, 0x41U, 0x7fU, 0x41U, 0x00U};
        source = data;
        break;
      }
      case 'L':
      {
        static const uint8_t data[5] = {0x7fU, 0x40U, 0x40U, 0x40U, 0x40U};
        source = data;
        break;
      }
      case 'N':
      {
        static const uint8_t data[5] = {0x7fU, 0x02U, 0x0cU, 0x10U, 0x7fU};
        source = data;
        break;
      }
      case 'O':
      {
        static const uint8_t data[5] = {0x3eU, 0x41U, 0x41U, 0x41U, 0x3eU};
        source = data;
        break;
      }
      case 'P':
      {
        static const uint8_t data[5] = {0x7fU, 0x09U, 0x09U, 0x09U, 0x06U};
        source = data;
        break;
      }
      case 'R':
      {
        static const uint8_t data[5] = {0x7fU, 0x09U, 0x19U, 0x29U, 0x46U};
        source = data;
        break;
      }
      case 'S':
      {
        static const uint8_t data[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
        source = data;
        break;
      }
      case 'T':
      {
        static const uint8_t data[5] = {0x01U, 0x01U, 0x7fU, 0x01U, 0x01U};
        source = data;
        break;
      }
      case 'U':
      {
        static const uint8_t data[5] = {0x3fU, 0x40U, 0x40U, 0x40U, 0x3fU};
        source = data;
        break;
      }
      case 'W':
      {
        static const uint8_t data[5] = {0x3fU, 0x40U, 0x38U, 0x40U, 0x3fU};
        source = data;
        break;
      }
      case 'X':
      {
        static const uint8_t data[5] = {0x63U, 0x14U, 0x08U, 0x14U, 0x63U};
        source = data;
        break;
      }
      case ':':
      {
        static const uint8_t data[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
        source = data;
        break;
      }
      case '-':
      {
        static const uint8_t data[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
        source = data;
        break;
      }
      default:
        break;
    }
  }

  memcpy(glyph, source, 5U);
}

static void DrawText2x(uint8_t x, uint8_t y, const char *text)
{
  uint8_t glyph[5];
  uint8_t column;
  uint8_t row;

  while (text != NULL && *text != '\0' && x <= (SSD1306_WIDTH - 12U))
  {
    GetGlyph(*text, glyph);
    for (column = 0U; column < 5U; ++column)
    {
      for (row = 0U; row < 7U; ++row)
      {
        if ((glyph[column] & (1U << row)) != 0U)
        {
          SetPixel((uint8_t)(x + column * 2U), (uint8_t)(y + row * 2U));
          SetPixel((uint8_t)(x + column * 2U + 1U), (uint8_t)(y + row * 2U));
          SetPixel((uint8_t)(x + column * 2U), (uint8_t)(y + row * 2U + 1U));
          SetPixel((uint8_t)(x + column * 2U + 1U),
                   (uint8_t)(y + row * 2U + 1U));
        }
      }
    }
    x = (uint8_t)(x + 12U);
    ++text;
  }
}

static uint16_t AbsoluteSpeed(int32_t value)
{
  if (value < 0)
  {
    value = -value;
  }
  if (value > 1000)
  {
    value = 1000;
  }
  return (uint16_t)value;
}

bool SSD1306_Init(I2C_HandleTypeDef *i2c)
{
  static const uint8_t initialization_commands[] = {
      0xaeU, 0xd5U, 0x80U, 0xa8U, 0x3fU, 0xd3U, 0x00U, 0x40U,
      0x8dU, 0x14U, 0x20U, 0x00U, 0xa1U, 0xc8U, 0xdaU, 0x12U,
      0x81U, 0x7fU, 0xd9U, 0xf1U, 0xdbU, 0x40U, 0xa4U, 0xa6U,
      0xafU,
  };

  display_i2c = i2c;
  memset(display_buffer, 0, sizeof(display_buffer));

  if (!WriteCommands(initialization_commands,
                     sizeof(initialization_commands)))
  {
    display_i2c = NULL;
    return false;
  }

  return FlushDisplay();
}

bool SSD1306_ShowControl(bool connected, int16_t left, int16_t right)
{
  char vertical_text[12];
  char horizontal_text[12];
  int32_t vertical_speed;
  int32_t horizontal_speed;

  if (display_i2c == NULL)
  {
    return false;
  }

  memset(display_buffer, 0, sizeof(display_buffer));

  if (!connected)
  {
    DrawText2x(22U, 24U, "No Xbox");
    return FlushDisplay();
  }

  vertical_speed = ((int32_t)left + (int32_t)right) / 2;
  horizontal_speed = ((int32_t)left - (int32_t)right) / 2;

  if (vertical_speed > 0)
  {
    (void)snprintf(vertical_text, sizeof(vertical_text), "UP:%04u",
                   (unsigned)AbsoluteSpeed(vertical_speed));
  }
  else if (vertical_speed < 0)
  {
    (void)snprintf(vertical_text, sizeof(vertical_text), "DOWN:%04u",
                   (unsigned)AbsoluteSpeed(vertical_speed));
  }
  else
  {
    (void)snprintf(vertical_text, sizeof(vertical_text), "STOP:%04u", 0U);
  }

  if (horizontal_speed > 0)
  {
    (void)snprintf(horizontal_text, sizeof(horizontal_text), "RIGHT:%04u",
                   (unsigned)AbsoluteSpeed(horizontal_speed));
  }
  else if (horizontal_speed < 0)
  {
    (void)snprintf(horizontal_text, sizeof(horizontal_text), "LEFT:%04u",
                   (unsigned)AbsoluteSpeed(horizontal_speed));
  }
  else
  {
    (void)snprintf(horizontal_text, sizeof(horizontal_text), "LR:%04u", 0U);
  }

  DrawText2x(4U, 7U, vertical_text);
  DrawText2x(4U, 39U, horizontal_text);
  return FlushDisplay();
}
