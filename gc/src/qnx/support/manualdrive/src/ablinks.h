/* Link header for application - AppBuilder 2.01  */

extern ApContext_t AbContext;

ApWindowLink_t base = {
	"base.wgtw",
	&AbContext,
	AbLinks_base, 0, 16
	};

ApDialogLink_t waypoint_file_dialog = {
	"waypoint_file_dialog.wgtd",
	&AbContext,
	AbLinks_waypoint_file_dialog, 32, 3
	};

static ApItem_t ApItems_file_menu[ 4 ] = {
	{ 1, 1, 0, NULL, 0, "load_waypoints_menuitem", "Load waypoints", NULL },
	{ 1, 16, 0, NULL, 4, "", "", NULL },
	{ 1, 1, 0, NULL, 0, "quit_menuitem", "Quit", NULL },
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
	36, 2, 3
	};

static ApItem_t ApItems_action_menu[ 5 ] = {
	{ 1, 1, 0, NULL, 0, "start_mission_menuitem", "Start mission", NULL },
	{ 1, 16, 0, NULL, 4, "", "", NULL },
	{ 1, 1, 0, NULL, 0, "reset_controllers_menuitem", "Reset controllers", NULL },
	{ 1, 1, 0, NULL, 0, "reboot_computers_menuitem", "Reboot computers", NULL },
	{ 0, 0, NULL, NULL, 0, NULL, NULL, NULL } };

ApMenuLink_t action_menu = {
	"action_menu",
	"",
	NULL,
	NULL,
	-2,
	ApItems_action_menu,
	& AbContext,
	AbLinks_action_menu,
	40, 3, 4
	};

static ApItem_t ApItems_mode_menu[ 8 ] = {
	{ 1, 64, 0, NULL, 0, "observe_menuitem", "Observe only", NULL },
	{ 1, 64, 0, NULL, 0, "remote_manual_menuitem", "Remote manual driving", NULL },
	{ 1, 64, 0, NULL, 0, "remote_semiauto_menuitem", "Remote semiauto driving", NULL },
	{ 1, 64, 0, NULL, 0, "auto_menuitem", "Automatic driving", NULL },
	{ 1, 16, 0, NULL, 4, "", "", NULL },
	{ 1, 64, 0, NULL, 0, "offline_semiauto_menuitem", "Offline semiauto testing", NULL },
	{ 1, 64, 0, NULL, 0, "offline_auto_menuitem", "Offline auto testing", NULL },
	{ 0, 0, NULL, NULL, 0, NULL, NULL, NULL } };

ApMenuLink_t mode_menu = {
	"mode_menu",
	"",
	NULL,
	NULL,
	-2,
	ApItems_mode_menu,
	& AbContext,
	AbLinks_mode_menu,
	45, 6, 7
	};

