/* Link header for application - AppBuilder 2.01  */

extern ApContext_t AbContext;

ApWindowLink_t base = {
	"base.wgtw",
	&AbContext,
	AbLinks_base, 0, 19
	};

ApDialogLink_t file_open_dlg = {
	"file_open_dlg.wgtd",
	&AbContext,
	AbLinks_file_open_dlg, 29, 6
	};

static ApItem_t ApItems_file_menu[ 5 ] = {
	{ 1, 1, 0, NULL, 0, "file_open", "Open...", NULL },
	{ 1, 1, 0, NULL, 0, "open_recent", "Open Recent", NULL },
	{ 1, 16, 0, NULL, 4, "", "", NULL },
	{ 1, 1, 0, NULL, 0, "file_quit", "Quit", NULL },
	{ 0, 0, NULL, NULL, 0, NULL, NULL, NULL } };

ApMenuLink_t file_menu = {
	"file_menu",
	"",
	NULL,
	NULL,
	-2,
	ApItems_file_menu,
	& AbContext,
	AbLinks_file_menu,
	35, 3, 4
	};

static ApItem_t ApItems_view_menu[ 3 ] = {
	{ 1, 4, 0, NULL, 0, "waypoint_toggle", "Waypoints", NULL },
	{ 1, 4, 0, NULL, 0, "obstacles_toggle", "Obstacles", NULL },
	{ 0, 0, NULL, NULL, 0, NULL, NULL, NULL } };

ApMenuLink_t view_menu = {
	"view_menu",
	"",
	NULL,
	NULL,
	-2,
	ApItems_view_menu,
	& AbContext,
	AbLinks_view_menu,
	40, 2, 2
	};

