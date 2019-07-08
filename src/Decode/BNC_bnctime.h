
#ifndef BNCTIME_H
#define BNCTIME_H

#include <string>

class bncTime {
 public:
  bncTime() {this->reset();}
  bncTime(int gpsw, double gpssec);
  bncTime(const std::string& isoString);

  /**
   * Set GPS time.
   * @param gpsw GPS week
   * @param gpssec GPS time of week in seconds
   * @return reference to current instance
   */
  bncTime& set(int gpsw, double gpssec);
  bncTime& set(int year, int month, int day, int hour, int min, double sec);
  bncTime& set(int year, int month, int day, double daysec);
  bncTime& setmjd(double daysec, int mjd);
  bncTime& setmjd(double mjddec);
  /**
   * Set GPS time relative to current time.
   * @param msec milliseconds of GPS week
   * @return reference to current instance
   */
  bncTime &set(int msec);
  /**
   * Set GPS time relative to current time.
   * @param msec milliseconds of current GPS day
   * @return reference to current instance
   */
  bncTime &setTOD(int msec);
  /**
   * Set GLONASS time relative to current time.
   * @param msec milliseconds of GLONASS day
   * @return reference to current instance
   */
  bncTime &setTk(int msec);
  /**
   * Set BDS time relative to current time.
   * @param msec milliseconds of BDS week
   * @return reference to current instance
   */
  bncTime &setBDS(int msec);

  /**
   * Set BDS time.
   * @param year 4 digit year
   * @param month month in year (1..12)
   * @param day day of month (1..31)
   * @param hour hour of day (0..23)
   * @param min minute of hour (0..59)
   * @param sec second of minute (0..59,60)
   * @return reference to current instance
   */
  bncTime &setBDS (int year, int month, int day, int hour, int min, double sec);
  bncTime &setBDS(int gpsw, double gpssec);

  void         reset() {_mjd = 0; _sec = 0.0;}
  unsigned int mjd()    const;
  double       daysec() const;
  /** Get GPS week.
   * @return GPS week number
   */
  unsigned int gpsw()   const;
  /** Get Galileo week.
   * @return Galileo week number
   */
  inline unsigned int galw() const { return gpsw()-1024; };
  /** Get Seconds in GPS week.
   * @return time of GPS week in seconds
   */
  double       gpssec() const;
  /** Get BDS/Beidou week.
   * @return BDS week number
   */
  unsigned int bdsw()   const;
  /** Get Seconds in BDS/Beidou week.
   * @return time of BDS week in seconds
   */
  double       bdssec() const;
  double       mjddec() const {return (_mjd + _sec / 86400.0);}
  void         civil_date (unsigned int& year, unsigned int& month,
                           unsigned int& day) const;
  void         civil_time (unsigned int& hour, unsigned int& min,
                           double& sec) const;
  bool         valid() const {return _mjd != 0 || _sec != 0.0;}
  bool         undef() const {return !valid();}
  bool         operator==(const bncTime &time1) const;
  bool         operator!=(const bncTime &time1) const;
  bool         operator<(const bncTime &time1) const;
  bool         operator>(const bncTime &time1) const;
  bool         operator<=(const bncTime &time1) const;
  bool         operator>=(const bncTime &time1) const;
  double       operator-(const bncTime &time1) const;
  bncTime      operator-(double sec) const;
  bncTime      operator+(double sec) const;
  bncTime&     operator+=(double sec);

  std::string timestr(unsigned numdec = 3, char sep = ':') const;
  std::string datestr(char sep = '-') const;
  operator std::string() const;

 private:
  unsigned int _mjd;
  double       _sec;
};

#endif

