#include "navconfig.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

namespace utiltool
{

ConfigInfo::Ptr ConfigInfo::config_info(new ConfigInfo());

/**
 * @brief  初始化函数
 * @note
 * @param  *config_file_path: 配置文件路径
 * @retval None
 */
bool ConfigInfo::open(const char *config_file_path)
{
  ifstream ifs_in(config_file_path, ios::in);
  if (!ifs_in)
  {
    cout << "Config File Lost" << endl;
    return false;
  }

  // 把配置读成key-value对
  while (!ifs_in.eof())
  {
    string line, key;
    getline(ifs_in, line);
    if (line.size() == 0)
    {
      continue;
    }
    else if (line.substr(0, 1) == "#" || line.substr(0, 1) == "*" || line.substr(0, 1) == "!")
    {
      continue;
    }
    auto data = TextSplit(line, ":");
    key = trim(data[0]);
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (data.size() > 1)
      storage[key] = trim(data[1]);
  }
  
  return true;
}

/**
 * @brief  获取单例模式下的唯一对象
 * @note
 * @retval
 */
ConfigInfo::Ptr ConfigInfo::GetInstance() { return config_info; }

} // namespace utiltool
