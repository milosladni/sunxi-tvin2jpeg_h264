#ifndef NDEBUG
#include <stdio.h>

/* define debug message colors */
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define DBG(format, ...) do { fprintf(stderr, KYEL"["KCYN"DEBUG"KYEL"] "KNRM format "\n", ##__VA_ARGS__); fflush(stdout); } while(0)
#define DBGF(format, ...) do { fprintf(stderr, KYEL"["KCYN"DEBUG"KYEL"] "KNRM "f(%s) -> " format "\n", __FUNCTION__, ##__VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(format, ...)
#define DBGF(format, ...)
#endif
