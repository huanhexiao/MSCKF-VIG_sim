/*
** NavAttitude.cc for Attitude Test in /home/fwt/myprogram/mscnav/submodules/tools/test
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 10 下午7:55:29 2019 little fang
** Last update Thu Jul 17 下午5:48:41 2019 little fang
*/

#include "navattitude.hpp"
#include "navlog.hpp"
#include "navstruct.hpp"
#include "navearth.hpp"
#include <sstream>
#include <fstream>
#include <iomanip>

using namespace utiltool;
using namespace attitude;
using namespace utiltool::constant;

void test(const Euler &euler)
{
    std::ostringstream osstr;
    osstr << std::fixed << std::setprecision(12) << "raw " << euler.transpose();
    auto q = Euler2Quaternion(euler);
    auto m = Euler2RotationMatrix(euler);
    osstr << std::fixed << std::setprecision(12) << " q " << (Quaternion2Euler(q)).transpose();
    osstr << std::fixed << std::setprecision(12) << " q_diff " << (Quaternion2Euler(q) - euler).transpose();
    osstr << std::fixed << std::setprecision(12) << " m " << (RotationMartix2Euler(m)).transpose();
    osstr << std::fixed << std::setprecision(12) << " m_diff " << (RotationMartix2Euler(m) - euler).transpose();
    LOG(ERROR) << osstr.str() << std::endl;
    return;
}

void test1()
{
    Euler euler{0.5, 1.5, 1.2};
    auto q = Euler2Quaternion(euler);
    auto m = Euler2RotationMatrix(euler);
    LOG(ERROR) << std::fixed << std::setprecision(16) << std::endl
               << "Quaternion " << std::endl
               << euler << std::endl
               << std::endl;

    LOG(ERROR) << std::fixed << std::setprecision(16) << std::endl
               << "Quaternion " << std::endl
               << q.coeffs() << std::endl
               << std::endl;

    LOG(ERROR) << std::fixed << std::setprecision(16) << std::endl
               << "Matrix " << std::endl
               << m << std::endl
               << std::endl;

    LOG(ERROR) << std::fixed << std::setprecision(16) << std::endl
               << "Euler from q " << std::endl
               << Quaternion2Euler(q) << std::endl
               << std::endl;

    LOG(ERROR) << std::fixed << std::setprecision(16) << std::endl
               << "Euler from m " << std::endl
               << RotationMartix2Euler(m) << std::endl
               << std::endl;
    LOG(ERROR) << earth::CalculateGravity({30 * deg2rad, 114 * deg2rad, 80}, false) << std::endl;
}

void test2()
{
    const Euler euler{0.01083286617_deg, -2.14248721480_deg, -75.74984266986_deg};
    auto q = attitude::Euler2Quaternion(euler);
    auto m = attitude::Euler2RotationMatrix(euler);
    LOG(INFO) << std::setprecision(18) << std::endl
              << euler.transpose() << std::endl;
    LOG(INFO) << std::setprecision(18) << std::endl
              << q.coeffs().transpose() << std::endl;
    LOG(INFO) << std::setprecision(18) << std::endl
              << m << std::endl;
}

int main(int argc, const char **argv)
{
    LogInit(argv[0], "./log/", google::INFO);
    test1();
    test2();
    // std::ifstream ifs_attlog(argv[1]);
    // if (!ifs_attlog.good())
    // {
    //     LOG(FATAL) << "file error";
    // }
    // int index = 1;
    // while (!ifs_attlog.eof())
    // {
    //     index *= -1;
    //     std::string line;
    //     std::getline(ifs_attlog, line);
    //     auto data = TextSplit(line, "\\s+");
    //     Euler euler(stod(data[0]), stod(data[1]), stod(data[2]));
    //     euler += Euler{M_PI * 2, M_PI * 2, M_PI * 2} * index;
    //     // test(euler);
    // }

    return 0;
}