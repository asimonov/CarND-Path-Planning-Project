//
// Created by Alexey Simonov on 05/08/2017.
//

#include "log_utils.h"
#include <iomanip>
#include <sstream>

class comma_numpunct : public std::__1::numpunct<char>
{
protected:
    virtual char do_thousands_sep() const
    {
      return ',';
    }

    virtual std::__1::string do_grouping() const
    {
      return "\03";
    }
};

unsigned long ts_ms()
{
  static auto glob_start_ts = std::__1::chrono::steady_clock::now().time_since_epoch().count();
  auto ts = std::__1::chrono::steady_clock::now().time_since_epoch().count();
  unsigned long ms = (ts-glob_start_ts) / 1000000;
  return ms;
}

std::string ts_ms_str()
{
  static std::__1::locale comma_locale(std::__1::locale(), new comma_numpunct());
  std::stringstream ss;
  ss.imbue(comma_locale);
  ss << std::__1::setw(8) << std::__1::fixed << ts_ms() << " ms: ";
  return ss.str();
}