// 24x24字符串显示函数 - GB2312版本
void OLED_ShowChinese24x24String(uint8_t x, uint8_t y, const char *text)
{
  uint8_t current_x = x;

  if (text == NULL) {
    return;
  }

  if (y > ((OLED_HEIGHT / 8) - 3)) {
    return;
  }

  while (*text != '\0') {
    // GB2312双字节编码
    if ((uint8_t)*text >= 0xA1) {  // GB2312汉字区首字节范围 0xA1-0xFE
      if (current_x > (OLED_WIDTH - 24)) {
        break;  // 剩余宽度不足, 终止显示
      }

      uint16_t gb_code = ((uint8_t)text[0] << 8) | (uint8_t)text[1];
      OLED_ShowChinese24x24(current_x, y, gb_code);

      text += 2;
      current_x += 24;
    } else {
      // ASCII或其他字符, 停止处理
      break;
    }
  }
}

// 16x16字符显示函数 - GB2312版本
void OLED_ShowChinese16x16(uint8_t x, uint8_t y, uint16_t gb_code)
{
  uint8_t i, j, k;

  if (x > (OLED_WIDTH - 16) || y > ((OLED_HEIGHT / 8) - 2)) {
    return;
  }

  // 遍历字库查找匹配的汉字
  for (i = 0; i < OLED_Chinese16x16_Count; i++) {
    if (OLED_Chinese16x16[i].code == gb_code) {
      const uint8_t *glyph = OLED_Chinese16x16[i].data;

      // 字模按行存储(每行2字节=16列), 需转换为每列8点的页数据
      for (j = 0; j < 2; j++) {  // 2页(每页8行)
        OLED_SetCursor(y + j, x);

        for (k = 0; k < 16; k++) {  // 每列16像素
          uint8_t column_byte = 0;

          for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t row_index = j * 8 + bit;   // 当前页内的第bit行
            uint8_t row_byte = glyph[row_index * 2 + (k / 8)];
            uint8_t bit_mask = 0x80 >> (k % 8);

            if (row_byte & bit_mask) {
              column_byte |= (1u << bit);
            }
          }

          OLED_WriteData(column_byte);
        }
      }
      return;
    }
  }
}

// 16x16字符串显示函数 - GB2312版本
void OLED_ShowChinese16x16String(uint8_t x, uint8_t y, const char *text)
{
  uint8_t current_x = x;

  if (text == NULL) {
    return;
  }

  if (y > ((OLED_HEIGHT / 8) - 2)) {
    return;
  }

  while (*text != '\0') {
    // GB2312双字节编码
    if ((uint8_t)*text >= 0xA1) {  // GB2312汉字区首字节范围 0xA1-0xFE
      if (current_x > (OLED_WIDTH - 16)) {
        break;  // 剩余宽度不足, 终止显示
      }

      uint16_t gb_code = ((uint8_t)text[0] << 8) | (uint8_t)text[1];
      OLED_ShowChinese16x16(current_x, y, gb_code);

      text += 2;
      current_x += 16;
    } else {
      // ASCII或其他字符, 停止处理
      break;
    }
  }
}
