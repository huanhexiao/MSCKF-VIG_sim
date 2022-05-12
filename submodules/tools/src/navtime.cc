#include "navtime.h"
#include <ctime>
#include <string.h>


namespace utiltool
{

const int NavTime::MAXSECONDOFDAY = 86400;
const int NavTime::MAXSECONDOFWEEK = 86400 * 7;

NavTime::NavTime() { NavTime(0, 0, 0, 0, 0, 0.0); }

NavTime NavTime::operator=(const NavTime &time)
{
  year_ = time.year_;
  month_ = time.month_;
  day_ = time.day_;
  hour_ = time.hour_;
  minute_ = time.minute_;
  second_ = time.second_;
  gpsweek_ = time.gpsweek_;
  second_of_day_ = time.second_of_day_;
  second_of_week_ = time.second_of_week_;
  doy_ = time.doy_;
  mjd_ = time.mjd_;
}

NavTime::NavTime(int year, int month, int day, int hour, int minute, double second)
    : year_(year), month_(month), day_(day), hour_(hour), minute_(minute), second_(second)
{
  Commontime2Gpstime();
  Commontime2Doytime();
}

NavTime::NavTime(int year, int doy, double second_of_day) : year_(year), doy_(doy), second_of_day_(second_of_day)
{
  Doytime2Commontime();
  Commontime2Gpstime();
}

NavTime::NavTime(int GPSWeek, double second_of_week) : gpsweek_(GPSWeek), second_of_week_(second_of_week)
{
  Gpstime2Commontime();
  Commontime2Doytime();
}

void NavTime::Gpstime2Commontime()
{
  GPSTime2ModifyJulianDay();
  ModifyJulianDay2Commontime();
}

void NavTime::Commontime2Gpstime()
{
  Commontime2ModifyJulianDay();
  ModifyJulianDay2GPSTime();
}

void NavTime::Commontime2Doytime()
{
  Commontime2ModifyJulianDay();
  MODIFYJULIANDAY mjdtemp = mjd_;
  month_ = 1;
  day_ = 1;
  hour_ = 0;
  minute_ = 0;
  second_ = 0;
  Commontime2ModifyJulianDay();
  doy_ = mjdtemp.day - mjd_.day + 1;
  mjd_ = mjdtemp;
  ModifyJulianDay2Commontime();
  second_of_day_ = hour_ * 3600 + minute_ * 60 + second_;
}

void NavTime::Doytime2Commontime()
{
  NavTime temp_time(year_, 1, 1, 0, 0, 0.0);
  mjd_.day = temp_time.MJD().day + doy_ - 1;
  mjd_.tod.sn = second_of_day_;
  mjd_.tod.tos = second_of_day_ - static_cast<int>(second_of_day_);
  ModifyJulianDay2Commontime();
}

/**
 * @brief  Convert Time to Print String
 * @note
 * @param  &format: "%04d-%02d-%02d %02d:%02d:%.1f", "%d %.1f", "%04d %03d %.1f"
 * @param  time_type: COMMONTIME, GPSTIME, DOYTIME
 * @retval Time string
 */
std::string NavTime::Time2String(const std::string &format, TimeType time_type) const
{
  char temp_time_str[256];
  memset(temp_time_str, 0x0, sizeof(temp_time_str));
  if (time_type == COMMONTIME)
  {
    sprintf(temp_time_str, format.c_str(), year_, month_, day_, hour_, minute_, second_);
  }
  else if (time_type == GPSTIME)
  {
    sprintf(temp_time_str, format.c_str(), gpsweek_, second_of_week_);
  }
  else
  {
    sprintf(temp_time_str, format.c_str(), year_, doy_, second_of_day_);
  }
  std::string temp_result = temp_time_str;
  return temp_result;
}
/**
 * @brief  operator += for Navtime and seconds
 * @note
 * @param  second:
 * @retval NavTime add seconds
 */
void NavTime::operator+=(double second)
{
  // second_of_week_ += second;
  // int temp_week = (int)second_of_week_/MAXSECONDOFWEEK;
  // second_of_week_-=temp_week * MAXSECONDOFWEEK;
  // gpsweek_ += temp_week;
  mjd_.tod.sn += static_cast<int>(second);
  mjd_.tod.tos += second - static_cast<int>(second);
  while (mjd_.tod.tos > 1)
  {
    mjd_.tod.sn += 1;
    mjd_.tod.tos -= 1;
  }
  while (mjd_.tod.sn > MAXSECONDOFDAY)
  {
    mjd_.day++;
    mjd_.tod.sn -= MAXSECONDOFDAY;
  }
  ModifyJulianDay2Commontime();
  ModifyJulianDay2GPSTime();
  Commontime2Doytime();
}
/**
 * @brief  operator -= for Navtime and seconds
 * @note
 * @param  second:
 * @retval
 */
void NavTime::operator-=(double second)
{
  mjd_.tod.tos -= second - static_cast<int>(second);
  while (mjd_.tod.tos < 0)
  {
    mjd_.tod.tos += 1.0;
    mjd_.tod.sn -= 1;
  }
  mjd_.tod.sn -= static_cast<int>(second);

  while (mjd_.tod.sn < 0.0)
  {
    mjd_.day--;
    mjd_.tod.sn += 86400.0;
  }
  ModifyJulianDay2Commontime();
  ModifyJulianDay2GPSTime();
  Commontime2Doytime();
}

/**
 * @brief  operator + for Navtime and seconds
 * @note
 * @param  second:
 * @retval
 */
NavTime NavTime::operator+(double second)
{
  NavTime result = *this;
  result += second;
  return result;
}
NavTime NavTime::operator+(int second) { return operator+((double)second); }

/**
 * @brief  operator - for Navtime and seconds
 * @note
 * @param  second:
 * @retval
 */
NavTime NavTime::operator-(double second)
{
  NavTime result = *this;
  result -= second;
  return result;
}
NavTime NavTime::operator-(int second) { return operator-((double)second); }

double NavTime::operator-(const NavTime& time) const
{
  int diffweek = gpsweek_ - time.GpsWeek();
  double diffsecond = second_of_week_ - time.SecondOfWeek();
  return (diffweek * 86400 * 7 + diffsecond);
}

/**
 * @brief  compare two Navtime with <
 * @note
 * @param  &time:
 * @retval bool, if this < time return true
 */
bool NavTime::operator<(const NavTime &time) const
{
  if (gpsweek_ < time.gpsweek_)
  {
    return true;
  }
  else if (gpsweek_ == time.gpsweek_ && second_of_week_ < time.second_of_week_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief  compare two Navtime with >
 * @note
 * @param  &time:
 * @retval bool, if this > time return true
 */
bool NavTime::operator>(const NavTime &time) const
{
  if (gpsweek_ > time.gpsweek_)
  {
    return true;
  }
  else if (gpsweek_ == time.gpsweek_ && second_of_week_ > time.second_of_week_)
  {
    return true;
  }
  else
  {
    return false;
  }
}
/**
 * @brief  compare two Navtime with >
 * @note
 * @param  &time:
 * @retval bool, if this == time return true
 */
bool NavTime::operator==(const NavTime &time) const
{

  if (gpsweek_ == time.gpsweek_ && second_of_week_ == time.second_of_week_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief  compare two Navtime with >=
 * @note
 * @param  time:
 * @retval bool, if this >= time return true
 */
bool NavTime::operator>=(const NavTime &time) const { return (*this > time || *this == time); }

/**
 * @brief  compare two Navtime with <=
 * @note
 * @param  time:
 * @retval bool, if this <= time return true
 */
bool NavTime::operator<=(const NavTime &time) const { return (*this < time || *this == time); }

/**
 * @brief
 * @note
 * @retval  NowTime with Navtime format
 */
NavTime NavTime::NowTime()
{
  time_t now_time;
  time(&now_time);
  struct tm *time2 = localtime(&now_time);
  NavTime time(time2->tm_year + 1900, time2->tm_mon + 1, time2->tm_mday, time2->tm_hour, time2->tm_min, time2->tm_sec);
  return time;
}

int NavTime::GpsWeek() const { return gpsweek_; }

int NavTime::Hour() const { return hour_; }

int NavTime::Minute() const { return minute_; }

int NavTime::Month() const { return month_; }

int NavTime::Year() const { return year_; }

int NavTime::Doy() const { return doy_; }

int NavTime::Day() const { return day_; }

double NavTime::Second() const { return second_; }

double NavTime::SecondOfDay() const { return second_of_day_; }

double NavTime::SecondOfWeek() const { return second_of_week_; }

MODIFYJULIANDAY NavTime::MJD() const { return mjd_; }

void NavTime::GPSTime2ModifyJulianDay()
{
  mjd_.tod.sn = int(second_of_week_);
  mjd_.tod.tos = second_of_week_ - mjd_.tod.sn;
  int temp_day = mjd_.tod.sn / 86400;
  mjd_.day = gpsweek_ * 7 + temp_day + 44244;
  mjd_.tod.sn -= temp_day * 86400;
}

void NavTime::Commontime2ModifyJulianDay()
{
  int y = year_;
  int m = month_;
  int temp;
  if (y >= 80 && y < 100)
  {
    y += 1900;
  }
  else if (y < 80)
  {
    y += 2000;
  }
  if (m <= 2)
  {
    y -= 1;
    m += 12;
  }
  temp = static_cast<int>(365.25 * y);
  temp += static_cast<int>(30.6001 * (m + 1));
  temp += day_;
  temp -= 679019;

  mjd_.day = temp;
  mjd_.tod.sn = hour_ * 3600 + minute_ * 60 + static_cast<int>(second_);
  mjd_.tod.tos = second_ - static_cast<int>(second_);
}

void NavTime::ModifyJulianDay2GPSTime()
{
  int gd;
  MODIFYJULIANDAY mjd;

  gd = mjd_.day - 44244;

  if (gd < 0)
  {
    mjd.tod.tos = 1 - mjd_.tod.tos;
    mjd.tod.sn = 86400 - mjd_.tod.sn - 1;
    mjd.day = 44244 - mjd_.day - 1;

    if (mjd.tod.tos >= 1.0)
    {
      mjd.tod.tos -= 1.0;
      mjd.tod.sn += 1;
    }
    if (mjd.tod.sn >= 86400)
    {
      mjd.tod.sn -= 86400;
      mjd.day += 1;
    }
  }
  else
  {
    mjd.day = mjd_.day - 44244;
    mjd.tod.sn = mjd_.tod.sn;
    mjd.tod.tos = mjd_.tod.tos;
  }

  gpsweek_ = mjd.day / 7;
  second_of_week_ = (mjd.day % 7) * 86400 + mjd.tod.sn + mjd.tod.tos;
}

void NavTime::ModifyJulianDay2Commontime()
{
  int a, b, c, d, e;
  a = mjd_.day + 2400001;
  b = a + 1537;
  c = static_cast<int>((b - 122.1) / 365.25);
  d = static_cast<int>(365.25 * c);
  e = static_cast<int>((b - d) / 30.6001);

  day_ = b - d - static_cast<int>(30.6001 * e);
  month_ = e - 1 - 12 * (e / 14);
  year_ = c - 4715 - ((7 + month_) / 10);
  hour_ = (int)(mjd_.tod.sn / 3600);
  minute_ = (mjd_.tod.sn % 3600) / 60;
  second_ = (mjd_.tod.sn % 3600) % 60 + mjd_.tod.tos;
}
} // namespace utiltool
