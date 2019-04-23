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
#include <stdio.h>
#include <mutex>

#ifdef ENABLE_LOG
#define LOG(level) \
	if(level >= Logger::LogLevel()) ; \
	else Logger::getInstance(level)
#else
#define LOG(...)
#endif


class Logger {
public:

	enum Level
	{
		ERROR = 0,
		WARNING,
		DEBUG,
		INFO,
		NONE
	};

	virtual ~Logger(){};

	static Logger& getInstance(const Level& level = INFO)
	{
		static Logger instance;
		messageLevel = level;
		return instance;
	};

	static Level& LogLevel();

	Logger& operator<<(const std::string& sMessage)
	{
		if(messageLevel >= LogLevel())
		{
			writeGuard.lock();
			outStream << "\n" << toString(messageLevel) <<  __FILE__ << "|" << __FUNCTION__
					  << "|" << __LINE__ << "|" << "\t";
			outStream << sMessage;

			std::cerr << outStream.str();
			outStream.str("");
			writeGuard.unlock();
		}
		return *this;
	}


protected:
	Logger();
	Logger(const Logger&) = delete;
	Logger& operator=(const Logger&) = delete;

	static const string& toString(const Level& level)
	{
		switch(level)
		{
			case INFO:
				return "<INFO>";
			case DEBUG:
				return "<DEBUG>";
			case WARNING:
				return "<WARN>";
			case ERROR:
				return "<ERROR>";
			default:
				return "";
		}
	}

	std::ostringstream outStream;
	static Level messageLevel;
	std::mutex writeGuard;
};


#endif /* LOGGER_H_ */
