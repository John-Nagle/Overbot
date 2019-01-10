/* Define header for application - AppBuilder 2.01  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_steering_position_meter;
#define ABW_steering_position_meter          AbGetABW( ABN_steering_position_meter )
extern const int ABN_steering_goal_meter;
#define ABW_steering_goal_meter              AbGetABW( ABN_steering_goal_meter )
extern const int ABN_refresh_timer;
#define ABW_refresh_timer                    AbGetABW( ABN_refresh_timer )
extern const int ABN_brake_pressure_meter;
#define ABW_brake_pressure_meter             AbGetABW( ABN_brake_pressure_meter )
extern const int ABN_brake_pressure_goal_meter;
#define ABW_brake_pressure_goal_meter        AbGetABW( ABN_brake_pressure_goal_meter )
extern const int ABN_steering_trend;
#define ABW_steering_trend                   AbGetABW( ABN_steering_trend )
extern const int ABN_brake_pressure_trend;
#define ABW_brake_pressure_trend             AbGetABW( ABN_brake_pressure_trend )
extern const int ABN_speed_meter;
#define ABW_speed_meter                      AbGetABW( ABN_speed_meter )
extern const int ABN_odometer_value;
#define ABW_odometer_value                   AbGetABW( ABN_odometer_value )
extern const int ABN_run_button;
#define ABW_run_button                       AbGetABW( ABN_run_button )
extern const int ABN_stop_button;
#define ABW_stop_button                      AbGetABW( ABN_stop_button )
extern const int ABN_message_text;
#define ABW_message_text                     AbGetABW( ABN_message_text )
extern const int ABN_throttle_meter;
#define ABW_throttle_meter                   AbGetABW( ABN_throttle_meter )
extern const int ABN_throttle_goal_meter;
#define ABW_throttle_goal_meter              AbGetABW( ABN_throttle_goal_meter )
extern const int ABN_throttle_trend;
#define ABW_throttle_trend                   AbGetABW( ABN_throttle_trend )
extern const int ABN_rpm_meter;
#define ABW_rpm_meter                        AbGetABW( ABN_rpm_meter )
extern const int ABN_shift_pane;
#define ABW_shift_pane                       AbGetABW( ABN_shift_pane )
extern const int ABN_shift_group;
#define ABW_shift_group                      AbGetABW( ABN_shift_group )
extern const int ABN_reverse_button;
#define ABW_reverse_button                   AbGetABW( ABN_reverse_button )
extern const int ABN_neutral_button;
#define ABW_neutral_button                   AbGetABW( ABN_neutral_button )
extern const int ABN_low_button;
#define ABW_low_button                       AbGetABW( ABN_low_button )
extern const int ABN_high_button;
#define ABW_high_button                      AbGetABW( ABN_high_button )
extern const int ABN_move_group;
#define ABW_move_group                       AbGetABW( ABN_move_group )
extern const int ABN_move_pane;
#define ABW_move_pane                        AbGetABW( ABN_move_pane )
extern const int ABN_move_hint_text;
#define ABW_move_hint_text                   AbGetABW( ABN_move_hint_text )
extern const int ABN_move_state_text;
#define ABW_move_state_text                  AbGetABW( ABN_move_state_text )
extern const int ABN_move_distance_value;
#define ABW_move_distance_value              AbGetABW( ABN_move_distance_value )
extern const int ABN_move_waypoint_number_value;
#define ABW_move_waypoint_number_value       AbGetABW( ABN_move_waypoint_number_value )
extern const int ABN_file_menu_button;
#define ABW_file_menu_button                 AbGetABW( ABN_file_menu_button )
extern const int ABN_action_menu_button;
#define ABW_action_menu_button               AbGetABW( ABN_action_menu_button )
extern const int ABN_mode_menu_button;
#define ABW_mode_menu_button                 AbGetABW( ABN_mode_menu_button )

/* 'waypoint_file_dialog' Dialog link */
extern const int ABN_waypoint_file_dialog;
#define ABW_waypoint_file_dialog             AbGetABW( ABN_waypoint_file_dialog )
extern const int ABN_waypoint_fileselector;
#define ABW_waypoint_fileselector            AbGetABW( ABN_waypoint_fileselector )
extern const int ABN_file_open_cancel_button;
#define ABW_file_open_cancel_button          AbGetABW( ABN_file_open_cancel_button )
extern const int ABN_file_open_ok_button;
#define ABW_file_open_ok_button              AbGetABW( ABN_file_open_ok_button )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

/* 'file_menu' Menu link */
extern const int ABN_file_menu;
extern const int ABN_load_waypoints_menuitem;
extern const int ABN_quit_menuitem;

/* 'action_menu' Menu link */
extern const int ABN_action_menu;
extern const int ABN_start_mission_menuitem;
extern const int ABN_reset_controllers_menuitem;
extern const int ABN_reboot_computers_menuitem;

/* 'mode_menu' Menu link */
extern const int ABN_mode_menu;
extern const int ABN_observe_menuitem;
extern const int ABN_remote_manual_menuitem;
extern const int ABN_remote_semiauto_menuitem;
extern const int ABN_auto_menuitem;
extern const int ABN_offline_semiauto_menuitem;
extern const int ABN_offline_auto_menuitem;

#define AB_OPTIONS "s:x:y:h:w:S:"
