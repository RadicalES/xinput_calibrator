/*
 * Copyright (c) 2009 Tias Guns
 * Copyright 2007 Peter Hutterer (xinput_ methods from xinput)
 * Copyright (c) 2011 Antoine Hue (invertX/Y)
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "calibrator/Evdev.hpp"

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <ctype.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 1
#endif
#ifndef EXIT_FAILURE
#define EXIT_FAILURE 0
#endif

// Constructor
CalibratorEvdev::CalibratorEvdev(const char* const device_name0,
                                 const XYinfo& axys0,
                                 XID device_id,
                                 const int thr_misclick,
                                 const int thr_doubleclick,
                                 const OutputType output_type,
                                 const char* geometry,
                                 const bool use_timeout,
                                 const char* output_filename)
  : Calibrator(device_name0, axys0, thr_misclick, thr_doubleclick, output_type, geometry, use_timeout, output_filename)
{
    
    cal_matrix.m[0] = 1.0;
    cal_matrix.m[1] = 0.0;
    cal_matrix.m[2] = 0.0;
    cal_matrix.m[3] = 0.0;
    cal_matrix.m[4] = 1.0;
    cal_matrix.m[5] = 0.0;
    cal_matrix.m[6] = 0.0;
    cal_matrix.m[7] = 0.0;
    cal_matrix.m[8] = 1.0;

    // init
    display = XOpenDisplay(NULL);
    if (display == NULL) {
        throw WrongCalibratorException("Evdev: Unable to connect to X server");
    }

    // normaly, we already have the device id
    if (device_id == (XID)-1) {
        devInfo = xinput_find_device_info(display, device_name, False);
        if (!devInfo) {
            XCloseDisplay(display);
            throw WrongCalibratorException("Evdev: Unable to find device");
        }
        device_id = devInfo->id;
    }

    dev = XOpenDevice(display, device_id);
    if (!dev) {
        XCloseDisplay(display);
        throw WrongCalibratorException("Evdev: Unable to open device");
    }

#ifndef HAVE_XI_PROP
    throw WrongCalibratorException("Evdev: you need at least libXi 1.2 and inputproto 1.5 for dynamic recalibration of evdev.");
#else

    
   // printf("EVDEV setting calibration matrix to default\n");
    if(!set_calibration(cal_matrix.m)) {
	XCloseDevice(display, dev);
        XCloseDisplay(display);
        throw WrongCalibratorException("Failed to reset calibration values\n");

    }
    
    get_calibration(cal_matrix.m);
    if(verbose) {
       printf("Calibrating EVDEV driver for \"%s\" id=%i\n", device_name, (int)device_id);
       printf("\tcurrent calibration values (from XInput): %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
                cal_matrix.m[0], cal_matrix.m[1], cal_matrix.m[2], cal_matrix.m[3], cal_matrix.m[4], cal_matrix.m[5],
		cal_matrix.m[6], cal_matrix.m[7], cal_matrix.m[8]);
   }
#endif // HAVE_XI_PROP

}
// protected pass-through constructor for subclasses
CalibratorEvdev::CalibratorEvdev(const char* const device_name0,
                                 const XYinfo& axys0,
                                 const int thr_misclick,
                                 const int thr_doubleclick,
                                 const OutputType output_type,
                                 const char* geometry,
                                 const bool use_timeout,
                                 const char* output_filename)
  : Calibrator(device_name0, axys0, thr_misclick, thr_doubleclick, output_type, geometry, output_filename) { }

// Destructor
CalibratorEvdev::~CalibratorEvdev () {
    XCloseDevice(display, dev);
    XCloseDisplay(display);
}

// From Calibrator but with evdev specific invertion option
// KEEP IN SYNC with Calibrator::finish() !!
//a = (screen_width * 6 / 8) / (click_3_X - click_0_X)
//c = ((screen_width / 8) - (a * click_0_X)) / screen_width
//e = (screen_height * 6 / 8) / (click_3_Y - click_0_Y)
//f = ((screen_height / 8) - (e * click_0_Y)) / screen_height
bool CalibratorEvdev::finish(int width, int height)
{
    if (get_numclicks() != NUM_POINTS) {
        return false;
    }

    // new axis origin and scaling
    // based on old_axys: inversion/swapping is relative to the old axis
    XYinfo new_axis(old_axys);


    // calculate average of clicks
    float x_min = (clicked.x[UL] + clicked.x[LL])/2.0;
    float x_max = (clicked.x[UR] + clicked.x[LR])/2.0;
    float y_min = (clicked.y[UL] + clicked.y[UR])/2.0;
    float y_max = (clicked.y[LL] + clicked.y[LR])/2.0;
  

    // the screen was divided in num_blocks blocks, and the touch points were at
    // one block away from the true edges of the screen.
    const float block_x = width/(float)num_blocks;
    const float block_y = height/(float)num_blocks;

    cal_matrix.height = height;
    cal_matrix.width = width;
    cal_matrix.a = (width * 6 / num_blocks) / (x_max - x_min);
    cal_matrix.c = ((width / num_blocks) - (cal_matrix.a * x_min)) / width;
    cal_matrix.e = (height * 6 / num_blocks) / (y_max - y_min);
    cal_matrix.f = ((height / num_blocks) - (cal_matrix.e * y_min)) / height;

    // rescale these blocks from the range of the drawn touchpoints to the range of the 
    // actually clicked coordinates, and substract/add from the clicked coordinates
    // to obtain the coordinates corresponding to the edges of the screen.
    float scale_x = (x_max - x_min)/(width - 2*block_x);
    x_min -= block_x * scale_x;
    x_max += block_x * scale_x;
    float scale_y = (y_max - y_min)/(height - 2*block_y);
    y_min -= block_y * scale_y;
    y_max += block_y * scale_y;
    
    cal_matrix.m[0] = cal_matrix.a;
    cal_matrix.m[2] = cal_matrix.c;
    cal_matrix.m[4] = cal_matrix.e;
    cal_matrix.m[5] = cal_matrix.f;

    return finish_data_matrix(cal_matrix);
}

//dummy
bool CalibratorEvdev::finish_data(const XYinfo &new_axys)
{
    return false;
}

// Activate calibrated data and output it
bool CalibratorEvdev::finish_data_matrix(MatrixInfo matrixInfo)
{
    bool success;

    //printf("\nDoing dynamic recalibration:\n");
    success = set_calibration(matrixInfo.m);

    // close
    XSync(display, False);

    if(verbose) {
    	printf("Final calibration matrix: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
                matrixInfo.m[0], matrixInfo.m[1], matrixInfo.m[2], matrixInfo.m[3], matrixInfo.m[4], matrixInfo.m[5],
		matrixInfo.m[6], matrixInfo.m[7], matrixInfo.m[8]);

    	printf("\t--> Making the calibration permanent <--\n");
    }
    output_xinput_cal_matrix(matrixInfo.m);
    return success;
}

bool CalibratorEvdev::set_swapxy(const int swap_xy)
{
    printf("\tSwapping X and Y axis...\n");

    // xinput set-int-prop "divername" "Evdev Axes Swap" 8 0
    int arr_cmd[1];
    arr_cmd[0] = swap_xy;

    bool ret = xinput_do_set_int_prop("Evdev Axes Swap", display, 8, 1, arr_cmd);

    if (verbose) {
        if (ret == true)
            printf("DEBUG: Successfully set swapped X and Y axes = %d.\n", swap_xy);
        else
            printf("DEBUG: Failed to set swap X and Y axes.\n");
    }

    return ret;
}

bool CalibratorEvdev::set_invert_xy(const int invert_x, const int invert_y)
{
    printf("\tInverting X and/or Y axis...\n");

    // xinput set-int-prop "divername" "Evdev Axis Inversion" 8 0 0
    int arr_cmd[2];
    arr_cmd[0] = invert_x;
    arr_cmd[1] = invert_y;

    bool ret = xinput_do_set_int_prop("Evdev Axis Inversion", display, 8, 2, arr_cmd);

    if (verbose) {
        if (ret == true)
            printf("DEBUG: Successfully set invert axis X=%d, Y=%d.\n", invert_x, invert_y);
        else
            printf("DEBUG: Failed to set axis inversion.\n");
    }

    return ret;
}

bool CalibratorEvdev::set_calibration(float matrix[])
{

      // XGetDeviceProperty vars
    Atom            act_type;
    int             act_format;
    unsigned long   nitems, bytes_after;
    Atom prop_float, prop_matrix;

    union DataFloat {
        unsigned char *c;
        float *f;
    } dataFloat;	

    if(verbose)
    printf("Setting calibration matrix to: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
                matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5],
		matrix[6], matrix[7], matrix[8]);


    prop_float = XInternAtom(display, "FLOAT", False);
    prop_matrix = XInternAtom(display, "libinput Calibration Matrix", False);

    if (!prop_float)
    {
	printf("Float atom not found. This server is too old.\n");
	return false;
    }
    
    if (!prop_matrix)
    {
	printf("libinput Calibration Matrix Atom not found. This "
                "server is too old\n");
	return false;

    }

    if(XGetDeviceProperty(display, dev, prop_matrix, 0, 9, False, prop_float,
                       &act_type, &act_format, &nitems, &bytes_after,
                       &dataFloat.c) != Success) {
	printf("XGetDeviceProperty for \"libinput Calibration Matrix\" failed. This "
                "server is too old\n");
	return false;
    }

    if (act_format != 32 || act_type != prop_float || nitems != 9 ||bytes_after != 0) {
	printf("XGetDeviceProperty for \"libinput Calibration Matrix\" returned invalid data\n");
	return false;
    }

    memcpy(dataFloat.f, matrix, nitems * sizeof(float));

    XChangeDeviceProperty(display, dev, prop_matrix, prop_float, act_format, PropModeReplace,
                      dataFloat.c, nitems);
    free(dataFloat.c);

    return true;
}

bool CalibratorEvdev::get_calibration(float matrix[])
{

      // XGetDeviceProperty vars
    Atom            act_type;
    int             act_format;
    unsigned long   nitems, bytes_after;
    Atom prop_float, prop_matrix;

    union DataFloat {
        unsigned char *c;
        float *f;
    } dataFloat;	


    prop_float = XInternAtom(display, "FLOAT", False);
    prop_matrix = XInternAtom(display, "libinput Calibration Matrix", False);

    if (!prop_float)
    {
	printf("Float atom not found. This server is too old.\n");
	return false;
    }
    
    if (!prop_matrix)
    {
	printf("libinput Calibration Matrix Atom not found. This "
                "server is too old\n");
	return false;

    }

    if(XGetDeviceProperty(display, dev, prop_matrix, 0, 9, False, prop_float,
                       &act_type, &act_format, &nitems, &bytes_after,
                       &dataFloat.c) != Success) {
	printf("XGetDeviceProperty for \"libinput Calibration Matrix\" failed. This "
                "server is too old\n");
	return false;
    }

    if (act_format != 32 || act_type != prop_float || nitems != 9 ||bytes_after != 0) {
	printf("XGetDeviceProperty for \"libinput Calibration Matrix\" returned invalid data\n");
	return false;
    }

    memcpy(matrix, dataFloat.f, nitems * sizeof(float));

    free(dataFloat.c);
    return true;
}


Atom CalibratorEvdev::xinput_parse_atom(Display *display, const char *name)
{
    Bool is_atom = True;
    int i;

    for (i = 0; name[i] != '\0'; i++) {
        if (!isdigit(name[i])) {
            is_atom = False;
            break;
        }
    }

    if (is_atom)
        return atoi(name);
    else
        return XInternAtom(display, name, False);
}

XDeviceInfo* CalibratorEvdev::xinput_find_device_info(
Display *display, const char *name, Bool only_extended)
{
    XDeviceInfo	*devices;
    XDeviceInfo *found = NULL;
    int		loop;
    int		num_devices;
    int		len = strlen(name);
    Bool	is_id = True;
    XID		id = (XID)-1;

    for (loop=0; loop<len; loop++) {
        if (!isdigit(name[loop])) {
            is_id = False;
            break;
        }
    }

    if (is_id) {
        id = atoi(name);
    }

    devices = XListInputDevices(display, &num_devices);

    for (loop=0; loop<num_devices; loop++) {
        if ((!only_extended || (devices[loop].use >= IsXExtensionDevice)) &&
            ((!is_id && strcmp(devices[loop].name, name) == 0) ||
             (is_id && devices[loop].id == id))) {
            if (found) {
                fprintf(stderr,
                        "Warning: There are multiple devices named \"%s\".\n"
                        "To ensure the correct one is selected, please use "
                        "the device ID instead.\n\n", name);
                return NULL;
            } else {
                found = &devices[loop];
            }
        }
    }

    return found;
}

// Set Integer property on  X
bool CalibratorEvdev::xinput_do_set_int_prop( const char * name,
                                         Display *display,
                                         int format,
                                         int argc,
                                         const int *argv )
{
#ifndef HAVE_XI_PROP
    return false;
#else

    Atom          prop;
    Atom          old_type;
    int           i;
    int           old_format;
    unsigned long act_nitems, bytes_after;

    union {
        unsigned char *c;
        short *s;
        long *l;
        Atom *a;
    } data;

    if (argc < 1)
    {
        fprintf(stderr, "Wrong usage of xinput_do_set_prop, need at least 1 arguments\n");
        return false;
    }

    prop = xinput_parse_atom(display, name);

    if (prop == None) {
        fprintf(stderr, "invalid property %s\n", name);
        return false;
    }

    if ( format == 0) {
        if (XGetDeviceProperty(display, dev, prop, 0, 0, False, AnyPropertyType,
                               &old_type, &old_format, &act_nitems,
                               &bytes_after, &data.c) != Success) {
            fprintf(stderr, "failed to get property type and format for %s\n",
                    name);
            return false;
        } else {
            format = old_format;
        }

        XFree(data.c);
    }

    data.c = (unsigned char*)calloc(argc, sizeof(long));

    for (i = 0; i < argc; i++) {
      switch (format) {
        case 8:
            data.c[i] = argv[i];
        case 16:
            data.s[i] = argv[i];
            break;
        case 32:
            data.l[i] = argv[i];
            break;

        default:
            fprintf(stderr, "unexpected size for property %s\n", name);
            return false;
      }
    }

    XChangeDeviceProperty(display, dev, prop, XA_INTEGER, format, PropModeReplace,
                      data.c, argc);
    free(data.c);
    return true;
#endif // HAVE_XI_PROP

}

bool CalibratorEvdev::output_xorgconfd(const XYinfo new_axys)
{
    const char* sysfs_name = get_sysfs_name();
    bool not_sysfs_name = (sysfs_name == NULL);
    if (not_sysfs_name)
        sysfs_name = "!!Name_Of_TouchScreen!!";

    if(output_filename == NULL || not_sysfs_name)
        printf("  copy the snippet below into '/etc/X11/xorg.conf.d/99-calibration.conf' (/usr/share/X11/xorg.conf.d/ in some distro's)\n");
    else
        printf("  writing xorg.conf calibration data to '%s'\n", output_filename);

    // xorg.conf.d snippet
    char line[MAX_LINE_LEN];
    std::string outstr;

    outstr += "Section \"InputClass\"\n";
    outstr += "	Identifier	\"calibration\"\n";
    sprintf(line, "	MatchProduct	\"%s\"\n", sysfs_name);
    outstr += line;
    sprintf(line, "	Option	\"Calibration\"	\"%d %d %d %d\"\n",
                new_axys.x.min, new_axys.x.max, new_axys.y.min, new_axys.y.max);
    outstr += line;
    sprintf(line, "	Option	\"SwapAxes\"	\"%d\"\n", new_axys.swap_xy);
    outstr += line;
    outstr += "EndSection\n";

    // console out
    printf("%s", outstr.c_str());
    if (not_sysfs_name)
        printf("\nChange '%s' to your device's name in the snippet above.\n", sysfs_name);
    // file out
    else if(output_filename != NULL) {
        FILE* fid = fopen(output_filename, "w");
        if (fid == NULL) {
            fprintf(stderr, "Error: Can't open '%s' for writing. Make sure you have the necessary rights\n", output_filename);
            fprintf(stderr, "New calibration data NOT saved\n");
            return false;
        }
        fprintf(fid, "%s", outstr.c_str());
        fclose(fid);
    }

    return true;
}

bool CalibratorEvdev::output_hal(const XYinfo new_axys)
{
    const char* sysfs_name = get_sysfs_name();
    bool not_sysfs_name = (sysfs_name == NULL);
    if (not_sysfs_name)
        sysfs_name = "!!Name_Of_TouchScreen!!";

    if(output_filename == NULL || not_sysfs_name)
        printf("  copy the policy below into '/etc/hal/fdi/policy/touchscreen.fdi'\n");
    else
        printf("  writing HAL calibration data to '%s'\n", output_filename);

    // HAL policy output
    char line[MAX_LINE_LEN];
    std::string outstr;

    sprintf(line, "<match key=\"info.product\" contains=\"%s\">\n", sysfs_name);
    outstr += line;
    sprintf(line, "  <merge key=\"input.x11_options.calibration\" type=\"string\">%d %d %d %d</merge>\n",
        new_axys.x.min, new_axys.x.max, new_axys.y.min, new_axys.y.max);
    outstr += line;
    sprintf(line, "  <merge key=\"input.x11_options.swapaxes\" type=\"string\">%d</merge>\n",
        new_axys.swap_xy);
    outstr += "</match>\n";
    // console out
    printf("%s", outstr.c_str());
    if (not_sysfs_name)
        printf("\nChange '%s' to your device's name in the config above.\n", sysfs_name);
    // file out
    else if(output_filename != NULL) {
        FILE* fid = fopen(output_filename, "w");
        if (fid == NULL) {
            fprintf(stderr, "Error: Can't open '%s' for writing. Make sure you have the necessary rights\n", output_filename);
            fprintf(stderr, "New calibration data NOT saved\n");
            return false;
        }
        fprintf(fid, "%s", outstr.c_str());
        fclose(fid);
    }

    return true;
}

bool CalibratorEvdev::output_xinput_cal_matrix(float m[])
{
    if(output_filename != NULL)
//        printf("  Install the 'xinput' tool and copy the command(s) below in a script that starts with your X session\n");
  //  else
        printf("  writing calibration script to '%s'\n", output_filename);

    // create startup script
    char line[MAX_LINE_LEN];
    std::string outstr;

    sprintf(line, "    xinput set-prop \"%s\" \"libinput Calibration Matrix\" %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,\n", device_name, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    outstr += line;

    // console out
    printf("%s", outstr.c_str());
    // file out
    if(output_filename != NULL) {
		FILE* fid = fopen(output_filename, "w");
		if (fid == NULL) {
			fprintf(stderr, "Error: Can't open '%s' for writing. Make sure you have the necessary rights\n", output_filename);
			fprintf(stderr, "New calibration data NOT saved\n");
			return false;
		}
		fprintf(fid, "%s", outstr.c_str());
		fclose(fid);
    }

    return true;
}

bool CalibratorEvdev::output_xinput(const XYinfo new_axys)
{
    if(output_filename == NULL)
        printf("  Install the 'xinput' tool and copy the command(s) below in a script that starts with your X session\n");
    else
        printf("  writing calibration script to '%s'\n", output_filename);

    // create startup script
    char line[MAX_LINE_LEN];
    std::string outstr;

    sprintf(line, "    xinput set-int-prop \"%s\" \"Evdev Axis Calibration\" 32 %d %d %d %d\n", device_name, new_axys.x.min, new_axys.x.max, new_axys.y.min, new_axys.y.max);
    outstr += line;
    sprintf(line, "    xinput set-int-prop \"%s\" \"Evdev Axes Swap\" 8 %d\n", device_name, new_axys.swap_xy);
    outstr += line;

    // console out
    printf("%s", outstr.c_str());
    // file out
    if(output_filename != NULL) {
		FILE* fid = fopen(output_filename, "w");
		if (fid == NULL) {
			fprintf(stderr, "Error: Can't open '%s' for writing. Make sure you have the necessary rights\n", output_filename);
			fprintf(stderr, "New calibration data NOT saved\n");
			return false;
		}
		fprintf(fid, "%s", outstr.c_str());
		fclose(fid);
    }

    return true;
}
