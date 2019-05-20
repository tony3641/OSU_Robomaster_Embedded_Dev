/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#include "bsp_key.h"

/**
  * @brief  detect key presse down event
  * @param  None
  * @retval event flag
  */
uint8_t key_scan(void)
{
  static uint8_t key_last = 0;
  static uint8_t key = 0;
  
  key_last = key;
  key = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
  if( (key == GPIO_PIN_RESET) && (key_last == GPIO_PIN_SET))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}