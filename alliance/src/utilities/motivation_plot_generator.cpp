#include <fstream>
#include <regex>
#include <ros/package.h>
#include "utilities/motivation_plot_generator.h"

namespace utilities
{
const std::string MotivationPlotConfigGenerator::TEMPORARY_PATH =
    "/tmp/alliance";
const std::string MotivationPlotConfigGenerator::MULTI_PLOT_TEMPLATE =
    MotivationPlotConfigGenerator::getTemplate("multiplot_template");
const std::string MotivationPlotConfigGenerator::SINGLE_PLOT_TEMPLATE =
    MotivationPlotConfigGenerator::getTemplate(
        "motivation_singleplot_template");
const std::string MotivationPlotConfigGenerator::DETAILED_MULTI_PLOT_TEMPLATE =
    MotivationPlotConfigGenerator::getTemplate("detailed_motivation_template");

void MotivationPlotConfigGenerator::generate(const alliance::Robot& robot)
{
  std::stringstream ss;
  int i(0), j(0);
  for (alliance::Robot::const_iterator it(robot.begin()); it != robot.end();
       it++)
  {
    alliance::BehaviourSetPtr behaviour_set(*it);
    ss << "  <row_" << i << ">\n";
    ss << "    <column_" << j << ">\n";
    std::string config(SINGLE_PLOT_TEMPLATE);
    config =
        std::regex_replace(config, std::regex("@robot_id@"), robot.getId());
    config = std::regex_replace(config, std::regex("@task_id@"),
                                behaviour_set->getTask()->getId());
    ss << config;
    ss << "    </column_" << j << ">\n";
    ss << "  </row_" << i << ">\n";
    i++;
  }
  std::ofstream generated_file(getFilename(robot.getId()));
  generated_file << std::regex_replace(MULTI_PLOT_TEMPLATE,
                                       std::regex("@plots@"), ss.str());
  generated_file.close();
}

void MotivationPlotConfigGenerator::generate(const alliance::Robot& robot,
                                             const alliance::Task& task)
{
  std::string config(DETAILED_MULTI_PLOT_TEMPLATE);
  config = std::regex_replace(config, std::regex("@robot_id@"), robot.getId());
  config = std::regex_replace(config, std::regex("@task_id@"), task.getId());
  std::ofstream generated_file(getFilename(robot.getId(), task.getId()));
  generated_file << config;
  generated_file.close();
}

std::string
MotivationPlotConfigGenerator::getTemplate(const std::string& filename)
{
  std::ifstream file;
  file.open(ros::package::getPath("alliance") + "/template/" + filename +
            ".xml.in");
  std::string content;
  std::getline(file, content, '\0');
  file.close();
  if (content.empty())
  {
    throw utilities::Exception("The " + ros::package::getPath("alliance") +
                               "/template/" + filename + ".xml.in" +
                               " does not exist or is empty.");
  }
  return content;
}

std::string
MotivationPlotConfigGenerator::getFilename(const std::string& robot_id,
                                           const char *root)
{
  std::string filename(robot_id);
  if (filename.size() > 1 && filename.at(0) == '/')
  {
    filename = filename.substr(1);
  }
  filename = std::regex_replace(filename, std::regex("/"), "-");
  return std::string(root) + "/" + filename + "-motivations.xml";
}

std::string
MotivationPlotConfigGenerator::getFilename(const std::string& robot_id,
                                           const std::string& task_id,
                                           const char* root)
{
  std::string filename(robot_id + "/" + task_id);
  if (filename.size() > 1 && filename.at(0) == '/')
  {
    filename = filename.substr(1);
  }
  filename = std::regex_replace(filename, std::regex("/"), "-");
  return std::string(root) + "/" + filename + "-detailed-motivation.xml";
}
}
