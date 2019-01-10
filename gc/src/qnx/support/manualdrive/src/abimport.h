/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApDialogLink_t waypoint_file_dialog;
extern ApWidget_t AbWidgets[ 36 ];

extern ApMenuLink_t file_menu;
extern ApMenuLink_t action_menu;
extern ApMenuLink_t mode_menu;

#ifdef __cplusplus
extern "C" {
#endif
int initoptions( int argc, char **argv );
int refresh_timer( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_open( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_window_close( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int steering_goal_meter_move( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int brake_pressure_goal_meter_move( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int stop_buttton_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int run_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_lost_focus( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int throttle_goal_meter_move( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int reverse_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int neutral_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int low_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int high_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int quit_menuitem_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int file_open_ok_button_pressed( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int start_mission_action( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int reset_controllers_action( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int reboot_computers_action( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int observe_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int semiauto_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int auto_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int remote_manual_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int offline_semiauto_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int offline_auto_selected( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
