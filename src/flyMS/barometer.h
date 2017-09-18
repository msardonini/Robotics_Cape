/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/


/*******************************************************************************
* BMP280 Barometer
*
* The robotics cape features a Bosch BMP280 barometer for measuring temperature, 
* pressure, and altitude.
*
* @ enum bmp_oversample_t 
* Setting given to initialize_barometer() which defines the oversampling done
* internally to the barometer. For example, if BMP_OVERSAMPLE_16 is used then
* the barometer will average 16 samples before updating the data registers.
* The more oversampling used, the slower the data registers will update. You
* should pick an oversample that provides an update rate slightly slower than 
* the rate at which you will be reading the barometer. 
* 
* @ enum bmp_filter_t
* Setting given to initialize_barometer() to configure the coefficient of the 
* internal first order filter. We recommend disabling the filter with
* BMP_FILTER_OFF and doing your own filtering with the discrete filter functions
* below.
*
* @ int initialize_barometer(bmp_oversample_t oversample, bmp_filter_t filter)
* powers on the barometer and initializes it with the given oversample and
* filter settings. returns 0 on success, otherwise -1.
*
* @ int power_off_barometer()
* Puts the barometer into a low power state, should be called at the end of
* your program before close. return 0 on success, otherwise -1.
*
* @ int read_barometer()
* Reads the newest temperature and pressure measurments from the barometer over
* the I2C bus. To access the data use the bmp_get_temperature_c(), 
* bmp_get_pressure_pa(), or bmp_get_altitude_m() functions. 
* returns 0 on success, otherwise -1.
*
* @ float bmp_get_temperature_c()
* This does not start an I2C transaction but simply returns the temperature in
* degrees celcius that was read by the last call to the read_barometer() 
* function.
*
* @ float bmp_get_pressure_pa()
* This does not start an I2C transaction but simply returns the pressure in
* pascals that was read by the last call to the read_barometer() function.
* 
* @ float bmp_get_altitude_m()
* This does not start an I2C transaction but simply returns the altitude in 
* meters based on the pressure received by the last call to the read_barometer()
* function. Assuming current pressure at sea level is the default 101325 Pa.
* Use set_sea_level_pressure_pa() if you know the current sea level pressure
* and desire more accuracy. 
* 
* @ int set_sea_level_pressure_pa(float pa)
* If you know the current sea level pressure for your region and weather, you 
* can use this to correct the altititude reading. This is not necessary if you
* only care about differential altitude from a starting point.
*******************************************************************************/
typedef enum bmp_oversample_t{
	BMP_OVERSAMPLE_1  =	(0x01<<2), // update rate 182 HZ
	BMP_OVERSAMPLE_2  =	(0x02<<2), // update rate 133 HZ
	BMP_OVERSAMPLE_4  =	(0x03<<2), // update rate 87 HZ
	BMP_OVERSAMPLE_8  =	(0x04<<2), // update rate 51 HZ
	BMP_OVERSAMPLE_16 =	(0x05<<2)  // update rate 28 HZ
} bmp_oversample_t;

typedef enum bmp_filter_t{
	BMP_FILTER_OFF = (0x00<<2),
	BMP_FILTER_2   = (0x01<<2),
	BMP_FILTER_4   = (0x02<<2),
	BMP_FILTER_8   = (0x03<<2),
	BMP_FILTER_16  = (0x04<<2)
}bmp_filter_t;

int initialize_barometer(bmp_oversample_t oversample, bmp_filter_t filter);
int power_off_barometer();
int read_barometer();
float bmp_get_temperature_c();
float bmp_get_pressure_pa();
float bmp_get_altitude_m();
int set_sea_level_pressure_pa(float pa);