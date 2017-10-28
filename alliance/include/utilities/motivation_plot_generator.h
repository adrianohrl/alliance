#ifndef _UTILITIES_MOTIVATION_PLOT_GENERATOR_H_
#define _UTILITIES_MOTIVATION_PLOT_GENERATOR_H_

#include "alliance/robot.h"

namespace utilities
{
class MotivationPlotConfigGenerator
{
public:
  static void generate(const alliance::Robot& robot);
  static void generate(const alliance::Robot& robot,
                       const alliance::Task& task);
  static void clear();

private:
  static const std::string ALLIANCE_PATH;
  static const std::string MULTI_PLOT_TEMPLATE_FILENAME;
  static const std::string SINGLE_PLOT_TEMPLATE_FILENAME;
  static const std::string MULTI_PLOT_TEMPLATE;
  static const std::string SINGLE_PLOT_TEMPLATE;
  static const std::string DETAILED_MULTI_PLOT_TEMPLATE;
  static std::string getAlliancePath();
  static std::string getTemplate(const std::string& filename);
  static std::string getFilename(const std::string& root,
                                 const alliance::Robot& robot);
  static std::string getFilename(const std::string& root,
                                 const alliance::Robot& robot,
                                 const alliance::Task& task);
};
}

#endif // _UTILITIES_MOTIVATION_PLOT_GENERATOR_H_
