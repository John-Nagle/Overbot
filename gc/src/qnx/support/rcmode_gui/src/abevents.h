/* Event header for application - AppBuilder 2.01  */

static const ApEventLink_t AbApplLinks[] = {
	{ 3, 0, 0L, 0L, 0L, &base, NULL, NULL, 0, NULL, 0, 0, 0, 0, },
	{ 0 }
	};

static const ApEventLink_t AbLinks_base[] = {
	{ 8, 2, 61521L, 0L, 0L, NULL, NULL, "base", 1010, base_arrowleft, 0, 0, 0, 0, },
	{ 8, 2, 61523L, 0L, 0L, NULL, NULL, "base", 1010, base_arrowright, 0, 0, 0, 0, },
	{ 8, 2, 61522L, 0L, 0L, NULL, NULL, "base", 1010, base_arrowup, 0, 0, 0, 0, },
	{ 8, 2, 61524L, 0L, 0L, NULL, NULL, "base", 1010, base_arrowdown, 0, 0, 0, 0, },
	{ 8, 2, 104L, 0L, 0L, NULL, NULL, "base", 1010, base_key_h, 0, 0, 0, 0, },
	{ 8, 2, 110L, 0L, 0L, NULL, NULL, "base", 1010, base_key_n, 0, 0, 0, 0, },
	{ 8, 2, 108L, 0L, 0L, NULL, NULL, "base", 1010, base_key_l, 0, 0, 0, 0, },
	{ 8, 2, 114L, 0L, 0L, NULL, NULL, "base", 1010, base_key_r, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_init_gear", 2009, button_init_gear_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "bigclickbutton", 2010, bigclickbutton_gotfocus, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "bigclickbutton", 2011, bigclickbutton_lostfocus, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "timer_periodic", 41002, timer_periodic_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "timer_periodic", 1012, timer_periodic_realized, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_init_brake", 2009, button_init_brake_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_init_throttle", 2009, button_init_throttle_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_init_steering", 2009, button_init_steering_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_send_throttle_and_brake", 2009, button_send_throttle_and_brake_activate, 0, 0, 0, 0, },
	{ 8, 0, 0L, 0L, 0L, NULL, NULL, "button_send_steering", 2009, button_send_steering_activate, 0, 0, 0, 0, },
	{ 0 }
	};

const char ApOptions[] = AB_OPTIONS;
