//
// Created by Alexey Simonov on 05/08/2017.
//

#include "log_utils.h"
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std;


class comma_numpunct : public std::numpunct<char>
{
protected:
    virtual char do_thousands_sep() const
    {
      return ',';
    }

    virtual std::string do_grouping() const
    {
      return "\03";
    }
};

unsigned long ts_ms()
{
  static auto glob_start_ts = std::chrono::steady_clock::now().time_since_epoch().count();
  auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
  unsigned long ms = (ts-glob_start_ts) / 1000000;
  return ms;
}

std::string ts_ms_str()
{
  static std::locale comma_locale(std::locale(), new comma_numpunct());
  std::stringstream ss;
  ss.imbue(comma_locale);
  ss << std::setw(8) << std::fixed << ts_ms() << " ms: ";
  return ss.str();
}

void dump_trajectory(const Trajectory& tr, std::string filename)
{
  ofstream f(filename.c_str(), ofstream::out);

  string line;
  vector<double> x = tr.getX();
  vector<double> y = tr.getY();
  int n = x.size();
  double dt = tr.getDt();
  for (int i=0;i<n;i++)
  {
    f<<(i*dt)<<" "<<x[i]<<" "<<y[i]<<endl;
  }

}
