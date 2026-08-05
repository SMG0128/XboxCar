#include "ssd1306_simple.h"

#include <string.h>

#define SSD1306_ADDRESS (0x3cU << 1U)
#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)
#define SSD1306_TIMEOUT_MS 20U

/* Text origins, chosen so the two 14 px lines clear each other. */
#define SSD1306_LINE1_X 4U
#define SSD1306_LINE1_Y 7U
#define SSD1306_LINE2_X 4U
#define SSD1306_LINE2_Y 39U
#define SSD1306_NO_XBOX_X 22U
#define SSD1306_NO_XBOX_Y 24U

static I2C_HandleTypeDef *display_i2c;

/* What we want on the panel. */
static uint8_t display_buffer[SSD1306_BUFFER_SIZE];

/*
 * What the panel is currently showing. The difference between the two is the
 * only thing that ever gets transmitted, which is what makes a static screen
 * free and a changed digit cheap.
 */
static uint8_t panel_buffer[SSD1306_BUFFER_SIZE];
static uint8_t dirty_mask;

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

  if (HAL_I2C_Mem_Write(display_i2c, SSD1306_ADDRESS, 0x40U,
                        I2C_MEMADD_SIZE_8BIT, display_buffer,
                        sizeof(display_buffer), SSD1306_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  memcpy(panel_buffer, display_buffer, sizeof(panel_buffer));
  dirty_mask = 0U;
  return true;
}

bool SSD1306_FlushPage(uint8_t page)
{
  uint8_t address_commands[6];

  if (display_i2c == NULL || page >= SSD1306_PAGE_COUNT)
  {
    return false;
  }

  address_commands[0] = 0x21U;
  address_commands[1] = 0x00U;
  address_commands[2] = 0x7fU;
  address_commands[3] = 0x22U;
  address_commands[4] = page;
  address_commands[5] = page;
  if (!WriteCommands(address_commands, sizeof(address_commands)))
  {
    return false;
  }

  if (HAL_I2C_Mem_Write(display_i2c, SSD1306_ADDRESS, 0x40U,
                        I2C_MEMADD_SIZE_8BIT,
                        &display_buffer[(uint16_t)page * SSD1306_WIDTH],
                        SSD1306_WIDTH, SSD1306_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  memcpy(&panel_buffer[(uint16_t)page * SSD1306_WIDTH],
         &display_buffer[(uint16_t)page * SSD1306_WIDTH], SSD1306_WIDTH);
  dirty_mask = (uint8_t)(dirty_mask & ~(uint8_t)(1U << page));
  return true;
}

/* Recomputes the dirty set after a render. */
static void MarkDirtyPages(void)
{
  uint8_t page;

  dirty_mask = 0U;
  for (page = 0U; page < SSD1306_PAGE_COUNT; ++page)
  {
    const uint16_t offset = (uint16_t)page * SSD1306_WIDTH;

    if (memcmp(&panel_buffer[offset], &display_buffer[offset],
               SSD1306_WIDTH) != 0)
    {
      dirty_mask = (uint8_t)(dirty_mask | (uint8_t)(1U << page));
    }
  }
}

uint8_t SSD1306_GetDirtyMask(void)
{
  return dirty_mask;
}

bool SSD1306_HasDirtyPages(void)
{
  return dirty_mask != 0U;
}

SSD1306FlushResult SSD1306_FlushDirtyPage(void)
{
  uint8_t page;

  if (display_i2c == NULL)
  {
    return SSD1306_FLUSH_FAILED;
  }

  for (page = 0U; page < SSD1306_PAGE_COUNT; ++page)
  {
    if ((dirty_mask & (uint8_t)(1U << page)) == 0U)
    {
      continue;
    }

    return SSD1306_FlushPage(page) ? SSD1306_FLUSH_SENT : SSD1306_FLUSH_FAILED;
  }

  return SSD1306_FLUSH_IDLE;
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
      case '/':
      {
        /* Needed by the centred "UP/DN" and "LT/RT" labels. */
        static const uint8_t data[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
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

  /*
   * The panel's RAM is not cleared by the reset sequence, so the shadow starts
   * deliberately different from the blank back buffer: the first flush must
   * transmit every page rather than assume the panel is already blank.
   */
  memset(panel_buffer, 0xFFU, sizeof(panel_buffer));
  dirty_mask = 0xFFU;

  if (!WriteCommands(initialization_commands,
                     sizeof(initialization_commands)))
  {
    display_i2c = NULL;
    return false;
  }

  return FlushDisplay();
}

bool SSD1306_RenderNoXbox(void)
{
  if (display_i2c == NULL)
  {
    return false;
  }

  memset(display_buffer, 0, sizeof(display_buffer));
  DrawText2x(SSD1306_NO_XBOX_X, SSD1306_NO_XBOX_Y, "No Xbox");
  MarkDirtyPages();
  return true;
}
bool SSD1306_RenderLines(const char *line1, const char *line2)
{
  if (display_i2c == NULL)
  {
    return false;
  }

  memset(display_buffer, 0, sizeof(display_buffer));
  if (line1 != NULL)
  {
    DrawText2x(SSD1306_LINE1_X, SSD1306_LINE1_Y, line1);
  }
  if (line2 != NULL)
  {
    DrawText2x(SSD1306_LINE2_X, SSD1306_LINE2_Y, line2);
  }
  MarkDirtyPages();
  return true;
}
