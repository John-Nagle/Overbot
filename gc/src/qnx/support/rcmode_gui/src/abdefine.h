/* Define header for application - AppBuilder 2.01  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_button_init_gear;
#define ABW_button_init_gear                 AbGetABW( ABN_button_init_gear )
extern const int ABN_meter_target_steering;
#define ABW_meter_target_steering            AbGetABW( ABN_meter_target_steering )
extern const int ABN_meter_current_steering;
#define ABW_meter_current_steering           AbGetABW( ABN_meter_current_steering )
extern const int ABN_bigclickbutton;
#define ABW_bigclickbutton                   AbGetABW( ABN_bigclickbutton )
extern const int ABN_numeric_steering;
#define ABW_numeric_steering                 AbGetABW( ABN_numeric_steering )
extern const int ABN_onOffButton_gear_initialized;
#define ABW_onOffButton_gear_initialized     AbGetABW( ABN_onOffButton_gear_initialized )
extern const int ABN_timer_periodic;
#define ABW_timer_periodic                   AbGetABW( ABN_timer_periodic )
extern const int ABN_meter_target_throttle;
#define ABW_meter_target_throttle            AbGetABW( ABN_meter_target_throttle )
extern const int ABN_meter_current_throttle;
#define ABW_meter_current_throttle           AbGetABW( ABN_meter_current_throttle )
extern const int ABN_numeric_throttle_and_brake;
#define ABW_numeric_throttle_and_brake       AbGetABW( ABN_numeric_throttle_and_brake )
extern const int ABN_meter_target_brake;
#define ABW_meter_target_brake               AbGetABW( ABN_meter_target_brake )
extern const int ABN_meter_current_brake;
#define ABW_meter_current_brake              AbGetABW( ABN_meter_current_brake )
extern const int ABN_numeric_steering_rate;
#define ABW_numeric_steering_rate            AbGetABW( ABN_numeric_steering_rate )
extern const int ABN_meter_target_steering_rate;
#define ABW_meter_target_steering_rate       AbGetABW( ABN_meter_target_steering_rate )
extern const int ABN_meter_current_steering_rate;
#define ABW_meter_current_steering_rate      AbGetABW( ABN_meter_current_steering_rate )
extern const int ABN_onOffButton_brake_initialized;
#define ABW_onOffButton_brake_initialized    AbGetABW( ABN_onOffButton_brake_initialized )
extern const int ABN_onOffButton_throttle_initialized;
#define ABW_onOffButton_throttle_initialized AbGetABW( ABN_onOffButton_throttle_initialized )
extern const int ABN_onOffButton_steering_initialized;
#define ABW_onOffButton_steering_initialized AbGetABW( ABN_onOffButton_steering_initialized )
extern const int ABN_button_init_brake;
#define ABW_button_init_brake                AbGetABW( ABN_button_init_brake )
extern const int ABN_button_init_throttle;
#define ABW_button_init_throttle             AbGetABW( ABN_button_init_throttle )
extern const int ABN_button_init_steering;
#define ABW_button_init_steering             AbGetABW( ABN_button_init_steering )
extern const int ABN_onOffButtonLostFocusEmergency;
#define ABW_onOffButtonLostFocusEmergency    AbGetABW( ABN_onOffButtonLostFocusEmergency )
extern const int ABN_button_send_throttle_and_brake;
#define ABW_button_send_throttle_and_brake   AbGetABW( ABN_button_send_throttle_and_brake )
extern const int ABN_button_send_steering;
#define ABW_button_send_steering             AbGetABW( ABN_button_send_steering )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

#define AB_OPTIONS "s:x:y:h:w:S:"
