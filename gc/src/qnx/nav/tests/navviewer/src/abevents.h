/* Event header for application - AppBuilder 2.01  */

static const ApEventLink_t AbApplLinks[] = {
	{ 3, 0, 0L, 0L, 0L, &base, NULL, NULL, 0, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_base[] = {
	{ 5, 0, 0L, 0L, 0L, &file_menu, NULL, "base", 2007, NULL, 0, 1, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18023, base_window_opening, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18024, base_window_closing, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "video_port", 24000, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) video_port_draw, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "redraw_timer", 41002, redraw_timer, 0, 0, 0, 0, },
	{ 5, 0, 0L, 0L, 0L, &file_menu, NULL, "file_menu_button", 2007, NULL, 0, 1, 0, 0, },
	{ 5, 0, 0L, 0L, 0L, &view_menu, NULL, "view_menu_button", 2007, NULL, 0, 1, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "vert_scrollbar", 29010, vert_scrollbar_moved, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "horiz_scrollbar", 29010, horiz_scrollbar_moved, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "center_button", 2007, center_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "zoom_scrollbar", 29010, zoom_scrollbar_moved, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "stop_button", 2007, stop_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "play_button", 2007, play_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "rewind_button", 2007, rewind_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "fast_forward_button", 2007, fast_forward_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "step_forward_button", 2007, step_forward_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "step_reverse_button", 2007, step_reverse_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "fast_reverse_button", 2007, fast_reverse_button_pushed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "frame_number_box", 53015, frame_number_changed, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_open_dlg[] = {
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_fileselector", 47012, file_open_dlg_fileselector, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_fileselector", 47013, file_open_dlg_fileselector, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_up_button", 2007, file_open_up_pushed, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_cancel", 2007, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_open", 2007, file_open_dlg_open, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_open", 2007, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_menu[] = {
	{ 4, 0, 0L, 0L, 0L, &file_open_dlg, NULL, "file_open", 2009, file_open_dlg_setup, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "open_recent", 2009, open_recent, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_quit", 2009, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_view_menu[] = {
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "waypoint_toggle", 2009, waypoint_toggle_changed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "obstacles_toggle", 2009, obstacles_toggle_changed, 0, 0, 0, 0, },
	{ 0 }
	};

const char ApOptions[] = AB_OPTIONS;
