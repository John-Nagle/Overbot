/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApWidget_t AbWidgets[ 5 ];


#ifdef __cplusplus
extern "C" {
#endif
int vorad_run_button( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
void vorad_port_draw( PtWidget_t *widget, PhTile_t *damage ) ;
int base_window_open( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_close( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int redraw_timer( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
