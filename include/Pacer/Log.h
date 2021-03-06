/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef LOG_H
#define LOG_H

#include <sstream>
#include <string>
#include <stdio.h>

inline std::string NowTime();

enum TLogLevel { logNONE, logERROR, logWARNING, logINFO, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};


class Log
{
public:
    Log();
    virtual ~Log();
    std::ostringstream& Get(TLogLevel level = logINFO);
public:
    static TLogLevel& ReportingLevel();
    static std::string ToString(TLogLevel level);
    static TLogLevel FromString(const std::string& level);
protected:
    std::ostringstream os;
private:
    Log(const Log&);
    Log& operator =(const Log&);
};

inline Log::Log()
{
}

inline std::ostringstream& Log::Get(TLogLevel level)
{
//    os << "- " << NowTime();
//    os << " " << ToString(level) << ": ";
    os << std::string(level > logDEBUG ? level - logDEBUG : 0, '\t');
    return os;
}

inline Log::~Log()
{
    os << std::endl;
#ifdef LOGGING
    FILE * pFile;
    pFile = fopen ("out.log","a");
    fprintf(pFile, "%s", os.str().c_str());
    fflush(pFile);
    fclose (pFile);
#else
    fprintf(stdout, "%s", os.str().c_str());
    fflush(stdout);
#endif
}

inline TLogLevel& Log::ReportingLevel()
{
    static TLogLevel reportingLevel = logDEBUG4;
    return reportingLevel;
}

inline std::string Log::ToString(TLogLevel level)
{
    static const char* const buffer[] = {"NONE","ERROR", "WARNING", "INFO", "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4"};
    return buffer[level];
}

inline TLogLevel Log::FromString(const std::string& level)
{
    if (level == "DEBUG4")
        return logDEBUG4;
    if (level == "DEBUG3")
        return logDEBUG3;
    if (level == "DEBUG2")
        return logDEBUG2;
    if (level == "DEBUG1")
        return logDEBUG1;
    if (level == "DEBUG")
        return logDEBUG;
    if (level == "INFO")
        return logINFO;
    if (level == "WARNING")
        return logWARNING;
    if (level == "ERROR")
        return logERROR;
    if (level == "NONE")
        return logNONE;
    Log().Get(logWARNING) << "Unknown logging level '" << level << "'. Using INFO level as default.";
    return logINFO;
}

typedef Log FILELog;

#define OUT_LOG(level) \
    if (level > FILELog::ReportingLevel()) ; \
    else Log().Get(level)

#define LOG(level) (level > FILELog::ReportingLevel())

#include <sys/time.h>

inline std::string NowTime()
{
    char buffer[11];
    time_t t;
    time(&t);
    tm r = {0};
    strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[100] = {0};
    sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000);
    return result;
}

void OUTLOG(const Ravelin::VectorNd& M, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SharedVectorNd& M, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::MatrixNd& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SharedConstMatrixNd& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Matrix3d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Pose3d& P, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Origin3d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Vector3d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Vector2d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SVector6d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::AAngled& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<double>& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<int>& z, std::string name,TLogLevel LL);
void OUTLOG(double x, std::string name,TLogLevel LL);
void OUTLOG(const std::string& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<std::string>& z, std::string name,TLogLevel LL);
extern std::string LOG_TYPE;

#endif // LOG_H
