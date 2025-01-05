/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2025 David J. Fiddes <D.J@fiddes.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef TESLAMODEL3_H
#define TESLAMODEL3_H

/**
 * \brief Encapsulate all of the Tesla Model 3 Inverter board specific
 * functionality
 */
class TeslaModel3
{
public:
    static void Initialize();
    static void CyclicFunction();
};


#endif // TESLAMODEL3_H
