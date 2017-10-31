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
  static std::string getFilename(const std::string& robot_id,
                                 const char* root = TEMPORARY_PATH.c_str());
  static std::string getFilename(const std::string& robot_id,
                                 const std::string& task_id,
                                 const char* root = TEMPORARY_PATH.c_str());

private:
  static const std::string TEMPORARY_PATH;
  static const std::string MULTI_PLOT_TEMPLATE_FILENAME;
  static const std::string SINGLE_PLOT_TEMPLATE_FILENAME;
  static const std::string MULTI_PLOT_TEMPLATE;
  static const std::string SINGLE_PLOT_TEMPLATE;
  static const std::string DETAILED_MULTI_PLOT_TEMPLATE;
  static std::string getTemplate(const std::string& filename);

};
}

#endif // _UTILITIES_MOTIVATION_PLOT_GENERATOR_H_
