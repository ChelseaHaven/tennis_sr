/* SPDX-License-Identifier: Apache-2.0
 * OGRE 1.12 assumes XRandR exposes at least one video mode. Headless Mutter
 * XWayland can report RANDR but no modes, causing a crash during GL setup.
 * Let OGRE take its existing non-RANDR fallback only in that case. This shim
 * is preloaded solely in RViz, and leaves usable RANDR displays untouched.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/extensions/Xrandr.h>

Bool XQueryExtension(Display *display, const char *name,
                     int *opcode, int *first_event, int *first_error)
{
    typedef Bool (*Query)(Display *, const char *, int *, int *, int *);
    Query original = (Query)dlsym(RTLD_NEXT, "XQueryExtension");
    Bool result = original(display, name, opcode, first_event, first_error);
    static __thread int checking = 0;
    if (result && !checking && strcmp(name, "RANDR") == 0) {
        checking = 1;
        XRRScreenConfiguration *config = XRRGetScreenInfo(display, DefaultRootWindow(display));
        int count = 0, usable = 0;
        if (config) {
            XRRConfigSizes(config, &count);
            for (int i = 0; i < count; ++i) {
                int rates = 0;
                XRRConfigRates(config, i, &rates);
                if (rates > 0) { usable = 1; break; }
            }
            XRRFreeScreenConfigInfo(config);
        }
        checking = 0;
        if (!usable) {
            static int reported = 0;
            if (!reported++) fprintf(stderr, "[rviz_x11] No XRandR video modes; using OGRE screen-size fallback.\n");
            return False;
        }
    }
    return result;
}
