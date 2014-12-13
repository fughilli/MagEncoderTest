/*
 * extsetup.h
 *
 *  Created on: Dec 8, 2014
 *      Author: Kevin Balke
 *
 *
 * Copyright (C) 2012-2014  Kevin Balke, Iyal Suresh, Alexander Fong
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef _EXT_SETUP_H_
#define _EXT_SETUP_H_

#include "Arduino.h"

void extsetup();
void extloop();

template <class T>
T clamp(T val, T cmin, T cmax)
{
    return min(max(cmin, val), cmax);
}

#endif // _EXT_SETUP_H_
