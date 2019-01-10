/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApWidget_t AbWidgets[ 25 ];


#ifdef __cplusplus
extern "C" {
#endif
int base_arrowleft( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_arrowright( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_arrowup( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_arrowdown( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_init_gear_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int timer_periodic_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int timer_periodic_realized( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int bigclickbutton_gotfocus( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int bigclickbutton_lostfocus( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_init_brake_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_init_throttle_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_init_steering_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_key_h( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_key_n( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_key_l( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int base_key_r( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_send_throttle_and_brake_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int button_send_steering_activate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
