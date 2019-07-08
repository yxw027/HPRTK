
#include <qdatetime.h>
#include <time.h>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>

#include "bnctime.h"
#include "bncutils.h"

using namespace std;

// Constructor
//////////////////////////////////////////////////////////////////////////////
bncTime::bncTime(int gpsw, double gpssec) {
  this->set(gpsw, gpssec);
}
  
// Constructor (from ISO String yyyy-mm-ddThh:mm:ss)
//////////////////////////////////////////////////////////////////////////////
bncTime::bncTime(const std::string& isoString) {
  if (!isoString.empty()) {
    QDateTime dt = QDateTime::fromString(isoString.c_str(), Qt::ISODate);
    this->set(dt.date().year(), dt.date().month(), dt.date().day(),
              dt.time().hour(), dt.time().minute(), 
              dt.time().second() + dt.time().msec()/1000.0);
  }
  else {
    this->reset();
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::set(int gpsw, double gpssec) {
  int  deltad;
  int  dow = 0;
  while ( gpssec >= 86400 ) {
    gpssec-=86400;
    dow++;
  }
  while ( gpssec <  0 ) {
    gpssec+=86400;
    dow--;
  }
  deltad = gpsw*7 + dow;
  _mjd = 44244 + deltad;
  _sec = gpssec;
  return *this;
}


//
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::setBDS(int gpsw, double gpssec) {
  int  deltad;
  int  dow = 0;
  gpssec += 14.0;
  gpsw   += 1356.0;
  while ( gpssec >= 86400 ) {
    gpssec-=86400;
    dow++;
  }
  while ( gpssec <  0 ) {
    gpssec+=86400;
    dow--;
  }
  deltad = gpsw*7 + dow;
  _mjd = 44244 + deltad;
  _sec = gpssec;
  return *this;
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime &bncTime::set(int msec) {
  int week;
  double sec;

  currentGPSWeeks(week, sec);
  if(msec/1000.0 < sec - 86400.0)
    ++week;
  return set(week, msec/1000.0);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime &bncTime::setTOD(int msec) {
  int week;
  double sec;

  currentGPSWeeks(week, sec);
  int intsec = sec;
  int day = intsec/(24*60*60);
  int tod = (intsec%(24*60*60))*1000;
  if(msec > 19*60*60*1000 && tod < 5*60*60*1000)
    --day;
  else if(msec < 5*60*60 && tod > 19*60*60*1000)
    ++day;
  msec += day*24*60*60*1000;
  if(msec < 0.0) {
    msec += 7*24*60*60*1000;
    --week;
  }

  return set(week, msec/1000.0);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime &bncTime::setTk(int msec) {
  int week;
  double sec;
  int intsec;

  currentGPSWeeks(week, sec);
  intsec = sec;
  updatetime(&week, &intsec, msec, 0); /* Moscow -> GPS */
  sec = intsec+(msec%1000)/1000.0;
  return set(week, sec);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime &bncTime::setBDS(int msec) {
  int week;
  double sec;

  msec += 14000;
  if(msec >= 7*24*60*60*1000)
    msec -= 7*24*60*60*1000;
  currentGPSWeeks(week, sec);
  if(msec/1000.0 < sec - 86400.0)
    ++week;
  return set(week, msec/1000.0);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::setmjd(double daysec, int mjd) {
  _sec = daysec;
  _mjd = mjd;
  while ( _sec >= 86400 ) {
    _sec-=86400;
    _mjd++;
  }
  while ( _sec <  0 ) {
    _sec+=86400;
    _mjd--;
  }
  return *this;
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::setmjd(double mjddec) {
  _mjd = static_cast<unsigned int>(mjddec);
  _sec = (mjddec - _mjd)*86400.0;
  return *this;
}

// 
//////////////////////////////////////////////////////////////////////////////
unsigned int bncTime::mjd() const {
  return _mjd;
}
 
//
//////////////////////////////////////////////////////////////////////////////
double bncTime::daysec() const {
  return _sec;
}

//
//////////////////////////////////////////////////////////////////////////////
unsigned int bncTime::gpsw() const {
  double   gsec;
  long     gpsw;
  jdgp(_mjd, gsec, gpsw);
  return (int)gpsw;
}

// 
//////////////////////////////////////////////////////////////////////////////
double bncTime::gpssec() const {
  double   gsec;
  long     gpsw;
  jdgp(_mjd, gsec, gpsw);
  return gsec + _sec;
}

//
//////////////////////////////////////////////////////////////////////////////
unsigned int bncTime::bdsw() const {
  double   gsec;
  long     gpsw;
  jdgp(_mjd, gsec, gpsw);
  if(gsec <= 14.0)
    gpsw -= 1;
  return (int)gpsw-1356;
}

// 
//////////////////////////////////////////////////////////////////////////////
double bncTime::bdssec() const {
  double   gsec;
  long     gpsw;
  jdgp(_mjd, gsec, gpsw);
  if(gsec <= 14.0)
    gsec += 7.0*24.0*60.0*60.0-14.0;
  else
    gsec -= 14.0;
  return gsec + _sec;
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator!=(const bncTime &time1) const {
  if ( fabs((*this) - time1) > 0.000000000001 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator==(const bncTime &time1) const {
  if ( fabs((*this) - time1) < 0.000000000001 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator>(const bncTime &time1) const {
  if ( ((*this) - time1) > 0.0 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator>=(const bncTime &time1) const {
  if ( ((*this) - time1) >= 0.0 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator<(const bncTime &time1) const {
  if ( ((*this) - time1) < 0.0 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bool bncTime::operator<=(const bncTime &time1) const {
  if ( ((*this) - time1) <= 0.0 ) {
    return true;
  }
  else {
    return false;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime bncTime::operator+(double sec) const {
  int     mjd    = this->mjd();
  double  daysec = this->daysec();
  daysec+=sec;
  return bncTime().setmjd(daysec, mjd);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime bncTime::operator-(double sec) const {
  return (*this) + (-sec);
}

// 
//////////////////////////////////////////////////////////////////////////////
double bncTime::operator-(const bncTime &time1) const {
  int mjdDiff = this->_mjd - time1._mjd;
  if ( mjdDiff != 0 ) {
    return mjdDiff * 86400.0 + this->_sec - time1._sec;
  }
  else {
    return this->_sec - time1._sec;
  }
}

bncTime& bncTime::operator+=(double sec) {
  _sec+=sec;

  while ( _sec >= 86400 ) {
    _sec-=86400;
    _mjd++;
  }
  while ( _sec < 0 ) {
    _sec+=86400;
    _mjd--;
  }

  return *this;
}

// 
//////////////////////////////////////////////////////////////////////////////
void bncTime::civil_date (unsigned int& year, unsigned int& month,
                          unsigned int& day) const {
  double day_d;
  long int yy, mm;
  jmt(_mjd, yy, mm, day_d);
  year  = yy;
  month = mm;
  day   = static_cast<unsigned int>(day_d);
}

// 
//////////////////////////////////////////////////////////////////////////////
void bncTime::civil_time(unsigned int &hour, unsigned int &min, 
                          double &sec) const {
  hour = static_cast<unsigned int>(_sec/3600.0);
  min  = static_cast<unsigned int>((_sec - hour*3600)/60.0);
  sec  = _sec - min*60 - hour*3600;
  if (sec==60.0) {
    min++;
    sec=0;
  }
  if (min==60) {
    hour++;
    min=0;
  }
}

// 
//////////////////////////////////////////////////////////////////////////////
string bncTime::timestr(unsigned numdec, char sep) const {
  ostringstream str;
  unsigned int hour, minute;
  double sec;
  this->civil_time(hour, minute, sec);
  unsigned sw;
  if (numdec == 0) {
    sw = 2;
  }
  else {
    sw = numdec + 3;
  }
  double chk = 0.5;
  for (unsigned int ii=0; ii<numdec; ii++) chk *= 0.1;
  if (sec > (60.0-chk)) {
    sec = 0;
    minute++;
    if (minute == 60) {
      minute = 0;
      hour++;
    }
  }
  str.setf(ios::fixed);
  str << setfill('0');
  str << setw(2)  << hour;
  if (sep) str << sep;
  str << setw(2)  << minute;
  if (sep) str << sep;
  str << setw(sw) << setprecision(numdec) << sec;
  return str.str();
}

// 
//////////////////////////////////////////////////////////////////////////////
string bncTime::datestr(char sep) const {
  unsigned int year, month, day;
  civil_date(year,month,day);
  ostringstream str;
  str.setf(ios::fixed);
  str << setfill('0');
  str << setw(4)  << year;
  if (sep) str << sep;
  str << setw(2)  << month;
  if (sep) str << sep;
  str << setw(2)  << day;
  return str.str();
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime::operator std::string() const {
  return datestr() + '_' + timestr();
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::set(int year, int month, int day, 
                      int hour, int min, double sec) {
  return set(year, month, day, hour*3600 + min*60 + sec);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::setBDS(int year, int month, int day, 
                      int hour, int min, double sec) {
  return set(year, month, day, hour*3600 + min*60 + sec+14.0);
}

// 
//////////////////////////////////////////////////////////////////////////////
bncTime& bncTime::set(int year, int month, int day, double daysec) {
  _sec = daysec;
  
  _mjd = (unsigned int)djul(year, month, day);
  
  while ( _sec >= 86400 ) {
    _sec-=86400;
    _mjd++;
  }
  while ( _sec <  0 ) {
    _sec+=86400;
    _mjd--;
  }

  return *this;
}
