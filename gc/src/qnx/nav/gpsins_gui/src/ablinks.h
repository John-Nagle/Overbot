/* Link header for application - AppBuilder 2.01  */

extern ApContext_t AbContext;

ApWindowLink_t base = {
	"base.wgtw",
	&AbContext,
	AbLinks_base, 0, 15
	};

ApDialogLink_t file_open_dialog = {
	"file_open_dialog.wgtd",
	&AbContext,
	AbLinks_file_open_dialog, 63, 3
	};

static ApItem_t ApItems_fileMenu[ 6 ] = {
	{ 1, 1, 0, NULL, 0, "fileMenuWaypointFile", "New waypoint file", NULL },
	{ 1, 1, 0, NULL, 0, "fileMenuCloseWaypointFile", "Close waypoint file", NULL },
	{ 1, 1, 0, NULL, 0, "fileMenuQuit", "Quit", NULL },
	{ 1, 16, 0, NULL, 4, "", "", NULL },
	{ 1, 1, 0, NULL, 0, "fileMenuAddWaypoint", "Add waypoint", NULL },
	{ 0, 0, NULL, NULL, 0, NULL, NULL, NULL } };

ApMenuLink_t fileMenu = {
	"fileMenu",
	"",
	NULL,
	NULL,
	-2,
	ApItems_fileMenu,
	& AbContext,
	AbLinks_fileMenu,
	69, 4, 5
	};

