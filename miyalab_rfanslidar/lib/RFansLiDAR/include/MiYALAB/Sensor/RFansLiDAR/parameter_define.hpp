/*
 * MIT License
 * 
 * Copyright (c) 2023 MiYA LAB(K.Miyauchi)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_PARAMETER_DEFINE_HPP__
#define __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_PARAMETER_DEFINE_HPP__

//-----------------------------
// enum
//-----------------------------
enum RFansModel{
    R_FANS_16,
    R_FANS_16M,
    R_FANS_32,
    R_FANS_32M
};

//-----------------------------
// Symbol
//-----------------------------
namespace MiYALAB{
namespace Sensor{
namespace RFansParams{
constexpr char MODEL_NAME[][32] = {
    "R-Fans-16",
    "R-Fans-16M",
    "R-Fans-32",
    "R-Fans-32M",
};

constexpr double RANGE_MAX = 200.0;
constexpr double LIDAR_RADIUS_SIZE = 0.056;

constexpr double DELTA_TIME_US[][32] = {
    {0.0, 13.32, 3.33, 16.65, 6.66, 19.98, 9.99, 23.31, 26.64, 39.96, 29.97, 43.29, 33.3, 46.62, 36.63, 49.95, 0.0 + 53.28, 13.32  + 53.28, 3.33  + 53.28, 16.65  + 53.28, 6.66 + 53.28, 19.98 + 53.28, 9.99 + 53.28, 23.31 + 53.28, 26.64 + 53.28, 39.96 + 53.28, 29.97 + 53.28, 43.29 + 53.28, 33.3 + 53.28, 46.62 + 53.28, 36.63 + 53.28, 49.95 + 53.28},
    {0.0, 13.32, 3.33, 16.65, 6.66, 19.98, 9.99, 23.31, 26.64, 39.96, 29.97, 43.29, 33.3, 46.62, 36.63, 49.95, 0.0 + 53.28, 13.32  + 53.28, 3.33  + 53.28, 16.65  + 53.28, 6.66 + 53.28, 19.98 + 53.28, 9.99 + 53.28, 23.31 + 53.28, 26.64 + 53.28, 39.96 + 53.28, 29.97 + 53.28, 43.29 + 53.28, 33.3 + 53.28, 46.62 + 53.28, 36.63 + 53.28, 49.95 + 53.28},
    {0.0, 6.25, 12.5, 18.75, 1.5625, 7.8125, 14.0625, 20.3125, 3.125, 9.375, 15.625, 21.875, 4.6875, 10.9375,  17.1875, 23.4375, 25.0, 31.25, 37.5, 43.75, 26.562, 32.812, 39.062, 45.312, 28.125, 34.375, 40.625, 46.875, 29.6875, 35.9375, 42.1875, 48.4375},
    {0.0, 6.25, 12.5, 18.75, 1.5625, 7.8125, 14.0625, 20.3125, 3.125, 9.375, 15.625, 21.875, 4.6875, 10.9375,  17.1875, 23.4375, 25.0, 31.25, 37.5, 43.75, 26.562, 32.812, 39.062, 45.312, 28.125, 34.375, 40.625, 46.875, 29.6875, 35.9375, 42.1875, 48.4375}
};

constexpr double VERTICAL_THETA[][32] = {
    {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15, -15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15},
    {-12, -15, -7.5, -9.5, -5. -6, -3, -4, -1, -2, 1, 0, 5, 3, 11, 8, -12, -15, -7.5, -9.5, -5. -6, -3, -4, -1, -2, 1, 0, 5, 3, 11, 8},
    {-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
    {-16.9, -15, -13.68, -12, -10.5, -9.5, -8.5, -7.5, -6.5, -6, -5.5, -5, -4.5, -4, -3.5, -3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 2, 3, 4, 5, 6.37, 8, 9.26, 11}
};

constexpr double HORIZONTAL_THETA[][32] = {
    {6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377},
    {6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377, 6.01, 3.377},
    {3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7},
    {3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7, 3.7, -6.35, 6.35, -3.7}
};
}
}
}

#endif // __MIYALAB_CPP_DRIVER_SENSOR_RFANS_LIDAR_PARAMETER_DEFINE_HPP__