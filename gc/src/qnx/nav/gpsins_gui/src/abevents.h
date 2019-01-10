/* Event header for application - AppBuilder 2.01  */

static const ApEventLink_t AbApplLinks[] = {
	{ 3, 0, 0L, 0L, 0L, &base, NULL, NULL, 0, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_base[] = {
	{ 5, 0, 0L, 0L, 0L, &fileMenu, NULL, "menuFile", 2007, NULL, 0, 1, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "heading", 24000, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) compassDraw, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "heading", 24003, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) compassInit, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "artHorizon", 24003, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) artHorizonInit, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "artHorizon", 24000, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) artHorizonDraw, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "positionView", 24000, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) positionViewDraw, 0, 0, 0, 0, },
	{ 8, 3, 0L, 0L, 0L, NULL, NULL, "positionView", 24003, (int(*)(PtWidget_t*,ApInfo_t*,PtCallbackInfo_t*)) positionViewInit, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "myTimer", 41002, myTimerActivate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "myTimer", 1012, myTimerRealized, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "buttonZoomOut", 2009, zoomOut, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "buttonClear", 2009, clear, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "buttonZoomIn", 2009, zoomIn, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "buttonZero", 2009, zero, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "onOffPause", 2009, togglePause, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "trendUnc", 1012, trendUncRealized, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_open_dialog[] = {
	{ 10, 0, 0L, 0L, 0L, NULL, NULL, "waypoint_file_cancel_button", 2007, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "create_waypoint_file_button", 2007, waypoint_file_dlg_open, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "create_waypoint_file_button", 2007, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_fileMenu[] = {
	{ 4, 0, 0L, 0L, 0L, &file_open_dialog, NULL, "fileMenuWaypointFile", 2009, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "fileMenuCloseWaypointFile", 2009, closewaypointfile, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "fileMenuQuit", 2009, quit, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "fileMenuAddWaypoint", 2009, addwaypoint, 0, 0, 0, 0, },
	{ 0 }
	};

