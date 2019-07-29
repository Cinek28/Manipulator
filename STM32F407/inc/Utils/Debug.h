/*
 * Debug.h
 *
 *  Created on: Jul 26, 2019
 *      Author: mrbr
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

typedef enum LEVEL
{
	ERR = 0,
	WARN,
	DBG,
	INFO
} LEVEL;

#define ENABLE_LOG

#ifdef ENABLE_LOG
static volatile LEVEL loggingLevel = DBG;
const char* toStr(LEVEL level)
{
	switch(level)
	{
		case(INFO):
			return "INFO";
		case(DBG):
			return "DBG";
		case(WARN):
			return "WARN";
		case(ERR):
			return "ERR";
		default:
			return "";
	}
};
#define _LOG(level, ...) \
	if(level > loggingLevel) ; \
	else \
	{ \
		printf("\r\n%s %s | %s[%d]:\t", \
				toStr(level), __FILE__ , \
				__FUNCTION__, __LINE__); \
		printf(__VA_ARGS__); \
	}
#else
#define _LOG(...)
#endif /* ENABLE_LOG */

#define _INFO(...) _LOG(INFO, __VA_ARGS__)
#define _DBG(...) _LOG(DBG, __VA_ARGS__)
#define _WARN(...) _LOG(WARN, __VA_ARGS__)
#define _ERR(...) _LOG(ERR, __VA_ARGS__)

#ifdef __cplusplus
}
#endif


#endif /* DEBUG_H_ */
