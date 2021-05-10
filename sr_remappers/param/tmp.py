#!/usr/bin/env python3
#
# Copyright 2011 Shadow Robot Company Ltd.
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
lapin = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'LFJ1', 'LFJ2',
         'LFJ3', 'LFJ4', 'LFJ5', 'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 'WRJ1', 'WRJ2']

for lap in lapin:
    if(lap == 'THJ1'):
        print("1.0 ", end=' ')
    else:
        print("0.0 ", end=' ')
print("")

for lap in lapin:
    if(lap == 'FFJ2'):
        print("1.0 ", end=' ')
    else:
        print("0.0 ", end=' ')
print("")

for lap in lapin:
    if(lap == 'MFJ2'):
        print("1.0 ", end=' ')
    else:
        print("0.0 ", end=' ')
print("")

for lap in lapin:
    if(lap == 'RFJ2'):
        print("1.0 ", end=' ')
    else:
        print("0.0 ", end=' ')
print("")

for lap in lapin:
    if(lap == 'LFJ2'):
        print("1.0 ", end=' ')
    else:
        print("0.0 ", end=' ')
