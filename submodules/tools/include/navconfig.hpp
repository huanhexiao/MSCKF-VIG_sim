/*
** navconfig.hpp for config  in /home/fwt/mypro/01-MSCNAV/include/util
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue May 14 8:17:51 2019 little fang
** Last update Tue Jul 22 下午5:01:37 2019 little fang
*/

#ifndef UTIL_CONFIG_H_
#define UTIL_CONFIG_H_

#include "navlog.hpp"
#include "navbase.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <string>
#include <regex>
#include <vector>
#include <exception>

namespace utiltool
{
class ConfigInfo
{
public:
  using Ptr = std::shared_ptr<ConfigInfo>;

private:
  ConfigInfo() {}
  ConfigInfo(ConfigInfo &&) = delete;
  ConfigInfo(const ConfigInfo &) = delete;
  ConfigInfo &operator=(ConfigInfo &&) = delete;
  ConfigInfo &operator=(const ConfigInfo &) = delete;
  static Ptr config_info;
  std::map<std::string, std::string> storage;

public:
  ~ConfigInfo() {}

public:
  static Ptr GetInstance();
  bool open(const char *config_file_path);
  template <typename T>
  T get(std::string key)
  {
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (storage.count(key) > 0)
    {
      try
      {
        double value = stod(storage[key]);
        return static_cast<T>(value);
      }
      catch (const std::exception &e)
      {
        std::cerr << e.what() << '\n';
      }
    }
    else
    {
      LOG(ERROR) << "The key of " << key << " does not exist, return a default value" << std::endl;
      std::cout << "The key of " << key << " does not exist, return a default value" << std::endl;
      std::cout << "Press enter to continue" << std::endl;
      getchar();
      return T(0x0);
    }
  }
  template <typename T>
  std::vector<T> get_array(std::string key)
  {
    std::vector<T> data;
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (storage.count(key) > 0)
    {
      try
      {
        auto text = TextSplit(storage[key], ",");
        for (auto index : text)
        {
          double value = stod(index);
          data.emplace_back(static_cast<T>(value));
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << e.what() << '\n';
      }
    }
    else
    {
      LOG(ERROR) << "The key of " << key << " does not exist, return a default value" << std::endl;
      std::cout << "The key of " << key << " does not exist, return a default value" << std::endl;
      std::cout << "Press enter to continue" << std::endl;
      getchar();
    }
    return data;
  }
};

template <>
inline std::string ConfigInfo::get<std::string>(std::string key)
{
  transform(key.begin(), key.end(), key.begin(), ::tolower);
  if (storage.count(key) > 0)
  {
    try
    {
      return std::string(storage[key]);
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  else
  {
    LOG(ERROR) << "The key of " << key << " does not exist, return a default value" << std::endl;
    std::cout << "The key of " << key << " does not exist, return a default value" << std::endl;
    std::cout << "Press enter to continue" << std::endl;
    getchar();
    return "";
  }
}

template <>
inline std::vector<std::string> ConfigInfo::get_array<std::string>(std::string key)
{
  std::vector<std::string> data;
  transform(key.begin(), key.end(), key.begin(), ::tolower);
  if (storage.count(key) > 0)
  {
    try
    {
      data = TextSplit(storage[key], ",");
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  else
  {
    LOG(ERROR) << "The key of " << key << " does not exist, return a default value" << std::endl;
    std::cout << "The key of " << key << " does not exist, return a default value" << std::endl;
    std::cout << "Press enter to continue" << std::endl;
    getchar();
  }
  return data;
}

} // namespace utiltool

#endif /* !CONFIG */
