#!/usr/bin/env python3
#
# Copyright 2011, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
#

# pylint: disable-all

with open("cyberglovetoshadowhand.map", 'r', encoding="utf8") as file_to_read:
    first = True
    matrix = []

    for line in file_to_read.readlines():
        if first:
            first = False
            continue
        else:
            line = line.strip("\n")
            line = line.split(" ")
            matrix.append(line)

matrix_t = []

for i in range(0, len(matrix[0])):
    matrix_t.append(list(range(0, len(matrix))))

for i in range(0, len(matrix)):
    for j in range(0, len(line)):
        matrix_t[j][i] = matrix[i][j]

with open("cyberglovetoshadowhand_transposed.map", 'w', encoding="utf8") as file_to_write:
    for line in matrix_t:
        for col in line:
            file_to_write.write(col + " ")
        file_to_write.write("\n")
    file_to_write.close()
