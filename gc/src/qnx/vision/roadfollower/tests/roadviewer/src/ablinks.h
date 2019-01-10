/* Link header for application - AppBuilder 2.01  */

extern ApContext_t AbContext;

ApWindowLink_t base = {
	"base.wgtw",
	&AbContext,
	AbLinks_base, 0, 6
	};

ApDialogLink_t file_open_dlg = {
	"file_open_dlg.wgtd",
	&AbContext,
	AbLinks_file_open_dlg, 4, 3
	};

static ApItem_t ApItems_file_menu[ 4 ] = {
	{ 1, 1, 0, NULL, 0, "file_open", "Open...", NULL },
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
	8, 2, 3
	};

