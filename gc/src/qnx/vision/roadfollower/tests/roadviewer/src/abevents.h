/* Event header for application - AppBuilder 2.01  */

static const ApEventLink_t AbApplLinks[] = {
	{ 3, 0, 0L, 0L, 0L, &base, NULL, NULL, 0, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_base[] = {
	{ 5, 0, 0L, 0L, 0L, &file_menu, NULL, "base", 2007, NULL, 0, 1, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18023, base_window_opening, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18024, base_window_closing, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "camera_run_button", 2007, camera_run_button, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "video_port", 24000, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) video_port_draw, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "redraw_timer", 41002, redraw_timer, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_open_dlg[] = {
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_cancel", 2007, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_open", 2007, file_open_dlg_open, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_open_dlg_open", 2007, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_menu[] = {
	{ 4, 0, 0L, 0L, 0L, &file_open_dlg, NULL, "file_open", 2009, NULL, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_quit", 2009, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

const char ApOptions[] = AB_OPTIONS;
