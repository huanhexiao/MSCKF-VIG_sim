/*
** navtime.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/inlcude
**
** Made by little fang
** Login   <fangwentao>
** basic struct of time
** Started on  Mon Dec 16 下午7:10:38 2018 little fang
** Last update Thu Jul 17 下午5:46:32 2019 little fang
*/

#ifndef NAVTIME_H_
#define NAVTIME_H_
#include <chrono>
#include <memory>
#include <string>
#include <time.h>

namespace utiltool
{

struct PRECISETIME
{
  PRECISETIME()
  {
    sn = 0;
    tos = 0.0;
  }

  long sn;
  double tos;
};
struct MODIFYJULIANDAY
{
  MODIFYJULIANDAY() { day = 0; }

  long day;
  PRECISETIME tod;
};
class NavTime
{
public:
  static const int MAXSECONDOFDAY;
  static const int MAXSECONDOFWEEK;

  enum TimeType
  {
    COMMONTIME,
    GPSTIME,
    DOYTIME
  };

public:
  NavTime();
  NavTime(int year, int month, int day, int hour, int minute, double second);
  NavTime(int year, int doy, double second_of_day);
  NavTime(int GPSWeek, double second_of_week);
  NavTime(const NavTime &time) = default;
  NavTime operator=(const NavTime &time);
  NavTime operator+(double second);
  NavTime operator-(double second);
  NavTime operator+(int second);
  NavTime operator-(int second);
  double operator-(const NavTime &time) const;
  void operator+=(double second);
  void operator-=(double second);
  bool operator<(const NavTime &time) const;
  bool operator>(const NavTime &time) const;
  bool operator<=(const NavTime &time) const;
  bool operator>=(const NavTime &time) const;
  bool operator==(const NavTime &time) const;
  ~NavTime() {}

public:
  static NavTime NowTime();
  std::string Time2String(const std::string &format = "%04d %02d %02d %02d %02d %04.1f", TimeType time_type = COMMONTIME) const;
  int Year() const;
  int Month() const;
  int Day() const;
  int Hour() const;
  int Minute() const;
  double Second() const;
  double SecondOfWeek() const;
  double SecondOfDay() const;
  int Doy() const;
  int GpsWeek() const;
  MODIFYJULIANDAY MJD() const;


public:
  using Ptr = std::shared_ptr<NavTime>;

private:
  void Commontime2Gpstime();
  void Commontime2Doytime();
  void Gpstime2Commontime();
  void Doytime2Commontime();

  void GPSTime2ModifyJulianDay();
  void Commontime2ModifyJulianDay();
  void ModifyJulianDay2GPSTime();
  void ModifyJulianDay2Commontime();

private:
  int year_;
  int month_;
  int day_;
  int hour_;
  int minute_;
  double second_;
  int gpsweek_;
  double second_of_day_;
  double second_of_week_;
  int doy_;
  MODIFYJULIANDAY mjd_;
};
} // namespace utiltool

#endif /* !NAVTIME_H_ */
