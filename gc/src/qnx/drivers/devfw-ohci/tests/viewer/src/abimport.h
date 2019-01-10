/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApDialogLink_t file_open_dlg;
extern ApWidget_t AbWidgets[ 9 ];

extern ApMenuLink_t file_menu;

#ifdef __cplusplus
extern "C" {
#endif
int file_open_dlg_open( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_opening( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_closing( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
void video_port_draw( PtWidget_t *widget, PhTile_t *damage ) ;
int camera_run_button( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int redraw_timer( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
