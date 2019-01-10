/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApDialogLink_t file_open_dlg;
extern ApWidget_t AbWidgets[ 35 ];

extern ApMenuLink_t file_menu;
extern ApMenuLink_t view_menu;

#ifdef __cplusplus
extern "C" {
#endif
int file_open_dlg_setup( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int file_open_dlg_open( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_opening( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_closing( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
void video_port_draw( PtWidget_t *widget, PhTile_t *damage ) ;
int redraw_timer( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int horiz_scrollbar_moved( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int vert_scrollbar_moved( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int center_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int zoom_scrollbar_moved( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int rewind_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int fast_reverse_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int stop_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int play_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int fast_forward_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int step_reverse_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int step_forward_button_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int frame_number_changed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int open_recent( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int file_open_dlg_fileselector( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int file_open_up_pushed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int waypoint_toggle_changed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int obstacles_toggle_changed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
