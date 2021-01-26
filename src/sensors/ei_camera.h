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

#ifndef EI_CAMERA
#define EI_CAMERA

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "../drivers/Portenta_Camera/portenta_camera.h"
#include "../ingestion-sdk-platform/portenta/ei_device_portenta.h"
#include "../edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "../repl/at_base64.h"
#include "../edge-impulse-sdk/dsp/numpy_types.h"

/* Constants --------------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240

/* Public function prototypes ---------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *buf);
bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height);
bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height);
bool ei_camera_inference_snapshot(size_t width, size_t height);
extern void ei_printf(const char *format, ...);

/**
 * @brief      Retrieves (cut-out) float RGB image data from the frame buffer
 *
 * @param[in]  offset        offset within cut-out image
 * @param[in]  length        number of bytes to read
 * @param[int] out_ptr       pointer to output buffre
 *
 * @retval     0 if successful
 *
 * @note       This function is called by the classifier to get float RGB image data
 */
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr);
//DEPRECATED
int ei_camera_scaled_get_data(size_t offset, size_t length, float *out_ptr);

/* Reference to object for external usage ---------------------------------- */
extern EiDevicePortenta EiDevice;

#endif // EI_CAMERA
