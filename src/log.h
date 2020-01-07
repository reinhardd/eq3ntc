#ifndef LOG_H
#define LOG_H

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>


#define L_Trace BOOST_LOG_TRIVIAL(trace)
#define L_Dbg   BOOST_LOG_TRIVIAL(trace)
#define L_Info  BOOST_LOG_TRIVIAL(info)
#define L_Warn  BOOST_LOG_TRIVIAL(warn)
#define L_Err   BOOST_LOG_TRIVIAL(error)
#define L_Fatal BOOST_LOG_TRIVIAL(fatal)

#endif // LOG_H
