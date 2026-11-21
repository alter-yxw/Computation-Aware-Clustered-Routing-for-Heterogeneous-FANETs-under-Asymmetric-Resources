/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-fuzzy-logic.h"
#include "ns3/log.h"
#include <limits>
#include <algorithm>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("CafbcrFuzzyLogic");

// ==================== TriangularMF Implementation ====================

TriangularMF::TriangularMF(double a, double b, double c)
  : m_a(a), m_b(b), m_c(c)
{
  NS_ASSERT_MSG(a < b && b < c, "Triangular MF requires a < b < c");
}

double TriangularMF::GetMembership(double value) const
{
  if (value <= m_a || value >= m_c) { return 0.0;}
  else if (value == m_b) {  return 1.0; }
  else if (value < m_b)  {  /* 上升段*/  return (value - m_a) / (m_b - m_a); }
  else /* value > m_b*/ {   /* 下降段*/  return (m_c - value) / (m_c - m_b); }
}

double TriangularMF::GetCenter() const
{
  return m_b;  // 三角形的峰值点作为中心
}

// ==================== TrapezoidalMF Implementation ====================

TrapezoidalMF::TrapezoidalMF(double a, double b, double c, double d)
  : m_a(a), m_b(b), m_c(c), m_d(d)
{
  NS_ASSERT_MSG(a <= b && b <= c && c <= d, "Trapezoidal MF requires a <= b <= c <= d");
}

double TrapezoidalMF::GetMembership(double value) const
{
  if (value <= m_a || value >= m_d) { return 0.0; }
  else if (value >= m_b && value <= m_c){ return 1.0;  /* 平台区域*/ }
  else if (value < m_b) { /* 左侧上升段*/
    if (m_b == m_a) return 1.0;
    return (value - m_a) / (m_b - m_a);
  }
  else  // value > m_c
  {
    // 右侧下降段
    if (m_d == m_c)  return 1.0;
    return (m_d - value) / (m_d - m_c);
  }
}

double TrapezoidalMF::GetCenter() const
{
  return (m_b + m_c) / 2.0;  // 平台中心作为中心值
}

// ==================== FuzzyVariable Implementation ====================

FuzzyVariable::FuzzyVariable()
  : m_name(""), m_minValue(0.0), m_maxValue(1.0), m_type(INPUT_VARIABLE)
{
  // 默认构造函数，用于std::map
}

FuzzyVariable::FuzzyVariable(const std::string& name, double minVal, double maxVal, FuzzyVariableType type)
  : m_name(name), m_minValue(minVal), m_maxValue(maxVal), m_type(type)
{
  NS_ASSERT_MSG(minVal < maxVal, "FuzzyVariable: minValue must be less than maxValue");
}

void FuzzyVariable::AddFuzzySet(const FuzzySet& fuzzySet)
{
  m_fuzzySets.push_back(fuzzySet);
}

void FuzzyVariable::Fuzzify(double crispValue)
{
  crispValue = std::max(m_minValue, std::min(m_maxValue, crispValue));  // 限制值在有效范围内
  for (auto& fuzzySet : m_fuzzySets)
  {
    if (fuzzySet.mf != nullptr)
    {
      fuzzySet.membership = fuzzySet.mf->GetMembership(crispValue); // 计算每个模糊集的隶属度
    }
    else
    {
      NS_LOG_WARN("Fuzzify: fuzzy set " << fuzzySet.name << " has null membership function");
      fuzzySet.membership = 0.0;
    }
  }
  NS_LOG_DEBUG("Fuzzified " << m_name << " = " << crispValue);
  for (const auto& fs : m_fuzzySets)
  {
    NS_LOG_DEBUG("  " << fs.name << ": " << fs.membership);
  }
}

double FuzzyVariable::Defuzzify() const
{
  // 使用重心法（Center of Gravity, COG）进行解模糊化
  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto& fuzzySet : m_fuzzySets)
  {
    if (fuzzySet.mf == nullptr)
    {
      NS_LOG_WARN("Defuzzify: fuzzy set " << fuzzySet.name << " has null membership function, skipping");
      continue;
    }
    double center = fuzzySet.mf->GetCenter();
    double membership = fuzzySet.membership;
    numerator += center * membership;
    denominator += membership;
  }

  if (denominator < 1e-6)
  {
    NS_LOG_WARN("Defuzzify: all memberships are zero, returning midpoint");
    return (m_minValue + m_maxValue) / 2.0;    // 如果所有隶属度都为0，返回中间值
  }
  double result = numerator / denominator;
  result = std::max(m_minValue, std::min(m_maxValue, result));  // 限制在有效范围内
  NS_LOG_DEBUG("Defuzzified " << m_name << " = " << result);

  return result;
}

FuzzySet* FuzzyVariable::GetFuzzySet(const std::string& setName)
{
  for (auto& fs : m_fuzzySets)
  {
    if (fs.name == setName)  { return &fs; }
  }
  return nullptr;
}

void FuzzyVariable::ResetMemberships()
{
  for (auto& fs : m_fuzzySets)
  {
    fs.membership = 0.0;
  }
}

// ==================== FuzzyRule Implementation ====================
FuzzyRule::FuzzyRule()
{
}

void FuzzyRule::AddAntecedent(const std::string& varName, const std::string& setName)
{
  m_antecedents[varName] = setName;
}

void FuzzyRule::AddConsequent(const std::string& varName, const std::string& setName)
{
  m_consequents[varName] = setName;
}

double FuzzyRule::CalculateFiringStrength(const std::map<std::string, FuzzyVariable>& inputVars) const
{
  if (m_antecedents.empty())  {  return 0.0; }

  double firingStrength = 1.0;  // 使用最小值算子（AND操作）计算激活强度

  for (const auto& antecedent : m_antecedents)
  {
    const std::string& varName = antecedent.first;
    const std::string& setName = antecedent.second;
    auto varIt = inputVars.find(varName);
    if (varIt == inputVars.end())
    {
      NS_LOG_WARN("Input variable " << varName << " not found in rule evaluation");
      return 0.0;
    }
    const FuzzyVariable& var = varIt->second;
    const FuzzySet* fuzzySet = nullptr;
    for (const auto& fs : var.GetAllFuzzySets())    // 查找对应的模糊集
    {
      if (fs.name == setName)
      {
        fuzzySet = &fs;
        break;
      }
    }

    if (fuzzySet == nullptr)
    {
      NS_LOG_WARN("Fuzzy set " << setName << " not found in variable " << varName);
      return 0.0;
    }
    firingStrength = std::min(firingStrength, fuzzySet->membership);    // 使用最小值算子
  }

  return firingStrength;
}

// ==================== MamdaniFuzzyEngine Implementation ====================

MamdaniFuzzyEngine::MamdaniFuzzyEngine()
{
}

void MamdaniFuzzyEngine::AddInputVariable(const FuzzyVariable& var)
{
  m_inputVariables[var.GetName()] = var;
}

void MamdaniFuzzyEngine::AddOutputVariable(const FuzzyVariable& var)
{
  m_outputVariables[var.GetName()] = var;
}

void MamdaniFuzzyEngine::AddRule(const FuzzyRule& rule)
{
  m_rules.push_back(rule);
}

void MamdaniFuzzyEngine::ClearRules()
{
  m_rules.clear();
}

FuzzyVariable* MamdaniFuzzyEngine::GetInputVariable(const std::string& name)
{
  auto it = m_inputVariables.find(name);
  if (it != m_inputVariables.end())  {  return &(it->second); }
  return nullptr;
}

FuzzyVariable* MamdaniFuzzyEngine::GetOutputVariable(const std::string& name)
{
  auto it = m_outputVariables.find(name);
  if (it != m_outputVariables.end()) {  return &(it->second); }

  return nullptr;
}

std::map<std::string, double> MamdaniFuzzyEngine::Infer(const std::map<std::string, double>& inputs)
{
  NS_LOG_FUNCTION(this);
  FuzzifyInputs(inputs);     // 步骤1: 模糊化所有输入
  EvaluateRules();           // 步骤2: 评估规则并聚合输出
  return DefuzzifyOutputs(); // 步骤3: 解模糊化所有输出
}

void MamdaniFuzzyEngine::FuzzifyInputs(const std::map<std::string, double>& inputs)
{
  for (auto& inputVar : m_inputVariables)
  {
    const std::string& varName = inputVar.first;
    FuzzyVariable& var = inputVar.second;
    auto inputIt = inputs.find(varName);
    if (inputIt != inputs.end())  { var.Fuzzify(inputIt->second); }
    else { NS_LOG_WARN("Input variable " << varName << " not provided, skipping"); }
  }
}

void MamdaniFuzzyEngine::EvaluateRules()
{
  for (auto& outputVar : m_outputVariables)  // 重置所有输出变量的隶属度
  {
    outputVar.second.ResetMemberships();
  }

  for (const auto& rule : m_rules) {// 评估每条规则
    double firingStrength = rule.CalculateFiringStrength(m_inputVariables);  // 计算规则的激活强度

    if (firingStrength > 0.0) {
      NS_LOG_DEBUG("Rule fired with strength: " << firingStrength);
      const auto& consequents = rule.GetConsequents();  // 应用规则后件（使用最大值算子进行聚合）
      for (const auto& consequent : consequents) {
        const std::string& varName = consequent.first;
        const std::string& setName = consequent.second;

        auto outputVarIt = m_outputVariables.find(varName);
        if (outputVarIt != m_outputVariables.end()) {
          FuzzyVariable& outputVar = outputVarIt->second;
          FuzzySet* fuzzySet = outputVar.GetFuzzySet(setName);
          if (fuzzySet != nullptr)
          {
            fuzzySet->membership = std::max(fuzzySet->membership, firingStrength);// 使用最大值算子聚合（Mamdani方法）
          }
        }
      }
    }
  }
}


std::map<std::string, double> MamdaniFuzzyEngine::DefuzzifyOutputs()
{
  std::map<std::string, double> outputs;
  for (auto& outputVar : m_outputVariables)
  {
    const std::string& varName = outputVar.first;
    FuzzyVariable& var = outputVar.second;
    double crispValue = var.Defuzzify();
    outputs[varName] = crispValue;
    NS_LOG_INFO("Output " << varName << " = " << crispValue);
  }

  return outputs;
}

} // namespace cafbcr
} // namespace ns3
