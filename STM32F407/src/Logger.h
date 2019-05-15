/*
 * Logger.h
 *
 *  Created on: 23.03.2019
 *      Author: Cinek
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <sstream>
#include <mutex>

class Logger {
public:
	typedef enum
	{
		NONE = 0,
		ERROR,
		WARNING,
		DEBUG,
		INFO
	} Level;

	Logger(){};

	virtual ~Logger()
	{
		if(messageLevel <= Logger::logLevel())
		{
			outStream << "\n"; 
			Logger::writeGuard.lock();
			std::cout << outStream.str();
			Logger::writeGuard.unlock();
		}
	};

	static const Level& logLevel()
	{
		return loggingLevel;
	}

	std::ostringstream& getStream(const Level& level)
	{
		if(level != Level::NONE)
			messageLevel = level;
		else
			messageLevel = Level::INFO;
		outStream << toString(messageLevel) << " |" 
		<< __FILE__ << "| " << __FUNCTION__ << "[" 
		<< __LINE__ << "]:" << "\t";
		return outStream;
	} 

	static const std::string toString(const Level& level)
	{
		switch(level)
		{
			case INFO:
				return "[INFO]";
			case DEBUG:
				return "[DEBUG]";
			case WARNING:
				return "[WARN]";
			case ERROR:
				return "[ERROR]";
			default:
				return "";
		}
	}

protected:
	Logger(const Logger&) = delete;
	Logger& operator=(const Logger&) = delete;

	std::ostringstream outStream;
	Level messageLevel;
	static Level loggingLevel;
	static std::mutex writeGuard;
};

std::mutex Logger::writeGuard;

#ifdef ENABLE_LOG
#define _LOG(level) \
	if(level > Logger::logLevel()) ; \
	else Logger().getStream(level)

Logger::Level Logger::loggingLevel = Logger::DEBUG;
#else
#define _LOG(...) \
	if(true) ; \
	else Logger().getStream(Logger::NONE)

Logger::Level Logger::loggingLevel = Logger::NONE;
#endif

#define _INFO _LOG(Logger::INFO)
#define _DBG _LOG(Logger::DEBUG)
#define _WARN _LOG(Logger::WARNING)
#define _ERR _LOG(Logger::ERROR)


#endif /* LOGGER_H_ */
