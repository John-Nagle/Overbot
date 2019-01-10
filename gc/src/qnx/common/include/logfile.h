#ifndef LOGFILE_H
#define LOGFILE_H
//
//	Logging utilities
//
//
//	buildlogfilename  -- build name of log file
//
//	Builds a log file name of the form
//
//	logdir/PREFIXyymmdd_hhmmss.SUFFIX
//
inline void buildlogfilename(char* filename, size_t filenamel, const char* logdir,  const char* prefix, const char* suffix)
{	assert(filenamel > 0);
	filename[0] = '\0';
	if (!logdir) return;											// no string if no dir
	assert(prefix);
	assert(suffix);
	assert(filename);	
	time_t now;													// current time
	time(&now);													// get current time
	struct tm* nowtime = localtime(&now);		// get time components
	snprintf(filename, filenamel,"%s/%s%04d%02d%02d_%02d%02d%02d.%s",
		logdir,prefix,
		nowtime->tm_year+1900, nowtime-> tm_mon+1, nowtime->tm_mday,
		nowtime->tm_hour, nowtime->tm_min, nowtime->tm_sec,
		suffix);
}
#endif // LOGFILE_H