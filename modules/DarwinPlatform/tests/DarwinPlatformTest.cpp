/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinPlatform.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include <iostream>

#include "Darwin.h"

int main(int argc, char *argv[]) {
    std::cout << "Starting CM730 Platform Test" << std::endl;
    
    // Connect to the darwin's CM730
    try {
        Darwin::Darwin darwin("/dev/ttyUSB0");
    }
    catch(std::runtime_error error) {
        
    }
}
