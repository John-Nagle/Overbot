/* Define header for application - AppBuilder 2.01  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_vorad_run_button;
#define ABW_vorad_run_button                 AbGetABW( ABN_vorad_run_button )
extern const int ABN_threat_meter;
#define ABW_threat_meter                     AbGetABW( ABN_threat_meter )
extern const int ABN_vorad_port;
#define ABW_vorad_port                       AbGetABW( ABN_vorad_port )
extern const int ABN_redraw_timer;
#define ABW_redraw_timer                     AbGetABW( ABN_redraw_timer )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

#define AB_OPTIONS "s:x:y:h:w:S:"
