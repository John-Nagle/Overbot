/* Define header for application - AppBuilder 2.01  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_video_port;
#define ABW_video_port                       AbGetABW( ABN_video_port )
extern const int ABN_redraw_timer;
#define ABW_redraw_timer                     AbGetABW( ABN_redraw_timer )
extern const int ABN_base_menubar;
#define ABW_base_menubar                     AbGetABW( ABN_base_menubar )
extern const int ABN_file_menu_button;
#define ABW_file_menu_button                 AbGetABW( ABN_file_menu_button )
extern const int ABN_view_menu_button;
#define ABW_view_menu_button                 AbGetABW( ABN_view_menu_button )
extern const int ABN_vert_scrollbar;
#define ABW_vert_scrollbar                   AbGetABW( ABN_vert_scrollbar )
extern const int ABN_horiz_scrollbar;
#define ABW_horiz_scrollbar                  AbGetABW( ABN_horiz_scrollbar )
extern const int ABN_center_button;
#define ABW_center_button                    AbGetABW( ABN_center_button )
extern const int ABN_speed_meter;
#define ABW_speed_meter                      AbGetABW( ABN_speed_meter )
extern const int ABN_heading_box;
#define ABW_heading_box                      AbGetABW( ABN_heading_box )
extern const int ABN_pitch_box;
#define ABW_pitch_box                        AbGetABW( ABN_pitch_box )
extern const int ABN_roll_box;
#define ABW_roll_box                         AbGetABW( ABN_roll_box )
extern const int ABN_Pitch;
#define ABW_Pitch                            AbGetABW( ABN_Pitch )
extern const int ABN_position_x_box;
#define ABW_position_x_box                   AbGetABW( ABN_position_x_box )
extern const int ABN_latitude_box;
#define ABW_latitude_box                     AbGetABW( ABN_latitude_box )
extern const int ABN_position_y_box;
#define ABW_position_y_box                   AbGetABW( ABN_position_y_box )
extern const int ABN_longitude_box;
#define ABW_longitude_box                    AbGetABW( ABN_longitude_box )
extern const int ABN_time_box;
#define ABW_time_box                         AbGetABW( ABN_time_box )
extern const int ABN_position_z_box;
#define ABW_position_z_box                   AbGetABW( ABN_position_z_box )
extern const int ABN_zoom_scrollbar;
#define ABW_zoom_scrollbar                   AbGetABW( ABN_zoom_scrollbar )
extern const int ABN_stop_button;
#define ABW_stop_button                      AbGetABW( ABN_stop_button )
extern const int ABN_play_button;
#define ABW_play_button                      AbGetABW( ABN_play_button )
extern const int ABN_rewind_button;
#define ABW_rewind_button                    AbGetABW( ABN_rewind_button )
extern const int ABN_fast_forward_button;
#define ABW_fast_forward_button              AbGetABW( ABN_fast_forward_button )
extern const int ABN_step_forward_button;
#define ABW_step_forward_button              AbGetABW( ABN_step_forward_button )
extern const int ABN_step_reverse_button;
#define ABW_step_reverse_button              AbGetABW( ABN_step_reverse_button )
extern const int ABN_fast_reverse_button;
#define ABW_fast_reverse_button              AbGetABW( ABN_fast_reverse_button )
extern const int ABN_frame_number_box;
#define ABW_frame_number_box                 AbGetABW( ABN_frame_number_box )

/* 'file_open_dlg' Dialog link */
extern const int ABN_file_open_dlg;
#define ABW_file_open_dlg                    AbGetABW( ABN_file_open_dlg )
extern const int ABN_file_open_dir_text;
#define ABW_file_open_dir_text               AbGetABW( ABN_file_open_dir_text )
extern const int ABN_file_open_dlg_fileselector;
#define ABW_file_open_dlg_fileselector       AbGetABW( ABN_file_open_dlg_fileselector )
extern const int ABN_file_open_up_button;
#define ABW_file_open_up_button              AbGetABW( ABN_file_open_up_button )
extern const int ABN_file_open_dlg_cancel;
#define ABW_file_open_dlg_cancel             AbGetABW( ABN_file_open_dlg_cancel )
extern const int ABN_file_open_dlg_open;
#define ABW_file_open_dlg_open               AbGetABW( ABN_file_open_dlg_open )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

/* 'file_menu' Menu link */
extern const int ABN_file_menu;
extern const int ABN_file_open;
extern const int ABN_open_recent;
extern const int ABN_file_quit;

/* 'view_menu' Menu link */
extern const int ABN_view_menu;
extern const int ABN_waypoint_toggle;
extern const int ABN_obstacles_toggle;

#define AB_OPTIONS "s:x:y:h:w:S:"
