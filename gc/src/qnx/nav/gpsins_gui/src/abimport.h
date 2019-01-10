/* Import (extern) header for application - AppBuilder 2.01  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApDialogLink_t file_open_dialog;
extern ApWidget_t AbWidgets[ 69 ];

extern ApMenuLink_t fileMenu;

#ifdef __cplusplus
extern "C" {
#endif
int start( int argc, char **argv );
int quit( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
void positionViewDraw( PtWidget_t *widget, PhTile_t *damage ) ;
int positionViewInit( PtWidget_t *widget ) ;
int myTimerActivate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int zoomIn( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int zoomOut( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int zero( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int clear( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int togglePause( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int trendUncRealized( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int artHorizonInit( PtWidget_t *widget ) ;
void artHorizonDraw( PtWidget_t *widget, PhTile_t *damage ) ;
void compassDraw( PtWidget_t *widget, PhTile_t *damage ) ;
int compassInit( PtWidget_t *widget ) ;
int myTimerRealized( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int addwaypoint( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int closewaypointfile( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
int waypoint_file_dlg_open( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
#ifdef __cplusplus
}
#endif
