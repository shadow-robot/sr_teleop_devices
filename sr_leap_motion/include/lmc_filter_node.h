/*
* Copyright 2021 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/
//!  Lowpass filter class
/*!
* This is a 2nd-order Butterworth LPF that is used to filter the hand x, y, z coordinates
* coming from the Leap Controller via Human.msg
* Refer to Julius O. Smith III, Intro. to Digital Filters with Audio Applications
*/

#ifndef LMC_FILTER_NODE_H
#define LMC_FILTER_NODE_H
class lpf
{
    public:
        lpf();
        explicit lpf(double);
        void setCutoff(double);
        void setInitialPos(double initial_msrmt, int b_idx, int dim);
        double filter(const double& new_msrmt, int f_idx, int dim);
        double getFilteredMsrmt(int f_idx, int dim);
        double c_;      // Related to the cutoff frequency of the filter.
                        // c = 1 results in a cutoff at 1/4 of the sampling rate.
                        // See bitbucket.org/AndyZe/pid if you want to get more sophisticated.
                        // Larger c --> trust the filtered data more, trust the measurements less.
    private:
        double prev_msrmts_ [6][3][3];
        double prev_filtered_msrmts_ [6][3][2];
};
#endif  // LMC_FILTER_NODE_H
