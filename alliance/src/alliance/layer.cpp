#include "alliance/layer.h"
#include <utilities/exception.h>

namespace alliance
{
Layer::Layer() {}

Layer::~Layer() {}

void Layer::initialize(const std::string& ns, const std::string& name)
{
  if (name.empty())
  {
    throw utilities::Exception("Layer name must not be empty.");
  }
  name_ = name;
  readParameters();
}

void Layer::setEvaluator(const SensoryEvaluatorPtr &evaluator)
{
  evaluator_ = evaluator;
}

void Layer::readParameters() {}

std::string Layer::getName() const { return name_; }

bool Layer::operator==(const Layer& layer) const
{
  return name_ == layer.name_;
}

bool Layer::operator!=(const Layer& layer) const
{
  return name_ != layer.name_;
}
}
