/* Event header for application - AppBuilder 2.01  */

static const ApEventLink_t AbApplLinks[] = {
	{ 3, 0, 0L, 0L, 0L, &base, NULL, NULL, 0, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_base[] = {
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18023, base_window_open, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 18024, base_window_close, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "base", 2011, base_lost_focus, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "steering_goal_meter", 74013, steering_goal_meter_move, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "refresh_timer", 41002, refresh_timer, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "brake_pressure_goal_meter", 74013, brake_pressure_goal_meter_move, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "run_button", 2007, run_button_pressed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "stop_button", 2007, stop_buttton_pressed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "throttle_goal_meter", 74013, throttle_goal_meter_move, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "reverse_button", 2007, reverse_button_pressed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "neutral_button", 2007, neutral_button_pressed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "low_button", 2007, low_button_pressed, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "high_button", 2007, high_button_pressed, 0, 0, 0, 0, },
	{ 5, 0, 0L, 0L, 0L, &file_menu, NULL, "file_menu_button", 2007, NULL, 0, 1, 0, 0, },
	{ 5, 0, 0L, 0L, 0L, &action_menu, NULL, "action_menu_button", 2007, NULL, 0, 1, 0, 0, },
	{ 5, 0, 0L, 0L, 0L, &mode_menu, NULL, "mode_menu_button", 2007, NULL, 0, 1, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_waypoint_file_dialog[] = {
	{ 10, 0, 0L, 0L, 0L, NULL, NULL, "file_open_cancel_button", 2007, NULL, 0, 0, 0, 0, },
	{ 9, 0, 0L, 0L, 0L, NULL, NULL, "file_open_ok_button", 2007, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "file_open_ok_button", 2007, file_open_ok_button_pressed, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_file_menu[] = {
	{ 4, 0, 0L, 0L, 0L, &waypoint_file_dialog, NULL, "load_waypoints_menuitem", 2009, NULL, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "quit_menuitem", 2009, quit_menuitem_selected, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_action_menu[] = {
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "start_mission_menuitem", 2009, start_mission_action, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "reset_controllers_menuitem", 2009, reset_controllers_action, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "reboot_computers_menuitem", 2009, reboot_computers_action, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_mode_menu[] = {
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "observe_menuitem", 2009, observe_selected, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "remote_manual_menuitem", 2009, remote_manual_selected, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "remote_semiauto_menuitem", 2009, semiauto_selected, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "auto_menuitem", 2009, auto_selected, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "offline_semiauto_menuitem", 2009, offline_semiauto_selected, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "offline_auto_menuitem", 2009, offline_auto_selected, 0, 0, 0, 0, },
	{ 0 }
	};

