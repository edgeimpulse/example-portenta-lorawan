/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EI_DEVICE_PORTENTA
#define EI_DEVICE_PORTENTA

/* Include ----------------------------------------------------------------- */
#include "ei_device_info.h"

/** Number of sensors used */
#if EI_DEVICE_PORTENTA_DISABLE_MICROPHONE == 1
#define EI_DEVICE_N_SENSORS		0
#else
#define EI_DEVICE_N_SENSORS		1
#endif

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
#define EI_DEVICE_N_RESOLUTIONS		3
#else
#define EI_DEVICE_N_RESOLUTIONS		2
#endif

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef bool (*c_callback_status)(void);
typedef int (*c_callback_get_data_output_baudrate)(ei_device_data_output_baudrate_t *baudrate);

typedef enum
{
	eiStateIdle 		= 0,
	eiStateErasingFlash,
	eiStateSampling,
	eiStateUploading,
	eiStateFinished

} tEiState;

/**
 * @brief      Class description and implementation of device specific
 * 			   characteristics
 */
class EiDevicePortenta : public EiDeviceInfo
{
private:
	ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
	ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS];
public:
	EiDevicePortenta(void);

	int get_id(uint8_t out_buffer[32], size_t *out_size);
	const char *get_id_pointer(void);
	int get_type(uint8_t out_buffer[32], size_t *out_size);
	const char *get_type_pointer(void);
	bool get_wifi_connection_status(void);
	bool get_wifi_present_status();
	bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
	bool get_snapshot_list(const ei_device_snapshot_resolutions_t **resolution_list, size_t *resolution_list_size,
						   const char **color_depth);
	int get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate);
	void set_state(tEiState state);

	c_callback get_id_function(void);
	c_callback get_type_function(void);
	c_callback_status get_wifi_connection_status_function(void);
	c_callback_status get_wifi_present_status_function(void);
	c_callback_get_data_output_baudrate get_data_output_baudrate_function(void);

};

/* Function prototypes ----------------------------------------------------- */
void ei_write_string(char *data, int length);
bool ei_user_invoke_stop(void);

/* Reference to object for external usage ---------------------------------- */
extern EiDevicePortenta EiDevice;

#endif