/* Define header for application - AppBuilder 2.01  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_camera_run_button;
#define ABW_camera_run_button                AbGetABW( ABN_camera_run_button )
extern const int ABN_video_port;
#define ABW_video_port                       AbGetABW( ABN_video_port )
extern const int ABN_redraw_timer;
#define ABW_redraw_timer                     AbGetABW( ABN_redraw_timer )

/* 'file_open_dlg' Dialog link */
extern const int ABN_file_open_dlg;
#define ABW_file_open_dlg                    AbGetABW( ABN_file_open_dlg )
extern const int ABN_file_open_dlg_fileselector;
#define ABW_file_open_dlg_fileselector       AbGetABW( ABN_file_open_dlg_fileselector )
extern const int ABN_file_open_dlg_cancel;
#define ABW_file_open_dlg_cancel             AbGetABW( ABN_file_open_dlg_cancel )
extern const int ABN_file_open_dlg_open;
#define ABW_file_open_dlg_open               AbGetABW( ABN_file_open_dlg_open )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

/* 'file_menu' Menu link */
extern const int ABN_file_menu;
extern const int ABN_file_open;
extern const int ABN_file_quit;

#define AB_OPTIONS "s:x:y:h:w:S:"
