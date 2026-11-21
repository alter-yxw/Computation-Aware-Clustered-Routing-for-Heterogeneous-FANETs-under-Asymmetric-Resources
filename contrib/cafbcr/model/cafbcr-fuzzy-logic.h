/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_FUZZY_LOGIC_H
#define CAFBCR_FUZZY_LOGIC_H

#include "ns3/object.h"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <cmath>

namespace ns3 {
namespace cafbcr {

/**
 * \brief Base class for membership functions.
 *
 * Abstract interface shared by all membership functions.
 */
class MembershipFunction
{
public:
  virtual ~MembershipFunction() {}

  /**
   * \brief Compute the membership degree of a crisp value.
   * \param value Crisp input value.
   * \return Membership degree in [0, 1].
   */
  virtual double GetMembership(double value) const = 0;

  /**
   * \brief Return the center of the membership function (used for defuzzification).
   * \return Center value.
   */
  virtual double GetCenter() const = 0;
};

/**
 * \brief Triangular membership function.
 *
 * Represents the MEDIUM state. Parameters satisfy a < b < c, where b is the peak.
 */
class TriangularMF : public MembershipFunction
{
public:
  /**
   * \brief Constructor.
   * \param a Left boundary.
   * \param b Peak.
   * \param c Right boundary.
   */
  TriangularMF(double a, double b, double c);

  double GetMembership(double value) const override;
  double GetCenter() const override;

private:
  double m_a;  ///< Left boundary.
  double m_b;  ///< Peak.
  double m_c;  ///< Right boundary.
};

/**
 * \brief Trapezoidal membership function.
 *
 * Represents the LOW and HIGH states. Parameters satisfy a < b < c < d, with
 * [b, c] as the plateau.
 */
class TrapezoidalMF : public MembershipFunction
{
public:
  /**
   * \brief Constructor.
   * \param a Lower-left boundary.
   * \param b Upper-left boundary (start of the plateau).
   * \param c Upper-right boundary (end of the plateau).
   * \param d Lower-right boundary.
   */
  TrapezoidalMF(double a, double b, double c, double d);

  double GetMembership(double value) const override;
  double GetCenter() const override;

private:
  double m_a;  ///< Lower-left boundary.
  double m_b;  ///< Upper-left boundary.
  double m_c;  ///< Upper-right boundary.
  double m_d;  ///< Lower-right boundary.
};

/**
 * \brief Fuzzy set.
 *
 * One value of a fuzzy linguistic variable, such as LOW, MEDIUM or HIGH.
 */
struct FuzzySet
{
  std::string name;                             ///< Fuzzy-set name (for example "LOW").
  std::shared_ptr<MembershipFunction> mf;       ///< Membership function.
  double membership;                            ///< Current membership degree in [0, 1].

  FuzzySet() : name(""), mf(nullptr), membership(0.0) {}
  FuzzySet(const std::string& n, std::shared_ptr<MembershipFunction> func)
    : name(n), mf(func), membership(0.0) {}
};

/**
 * \brief Fuzzy variable type.
 */
enum FuzzyVariableType
{
  INPUT_VARIABLE,   ///< Input variable.
  OUTPUT_VARIABLE   ///< Output variable.
};

/**
 * \brief Fuzzy variable.
 *
 * A linguistic variable with one or more fuzzy sets.
 */
class FuzzyVariable
{
public:
  /**
   * \brief Constructor.
   * \param name Variable name.
   * \param minVal Minimum value.
   * \param maxVal Maximum value.
   * \param type Input or output variable.
   */
  FuzzyVariable();
  FuzzyVariable(const std::string& name, double minVal, double maxVal, FuzzyVariableType type);

  /**
   * \brief Add a fuzzy set.
   * \param fuzzySet Fuzzy set to add.
   */
  void AddFuzzySet(const FuzzySet& fuzzySet);

  /**
   * \brief Fuzzify a crisp value into membership degrees for every fuzzy set.
   * \param crispValue Crisp input value.
   */
  void Fuzzify(double crispValue);

  /**
   * \brief Defuzzify the fuzzy output into a crisp value (centroid method).
   * \return Crisp output value.
   */
  double Defuzzify() const;

  /**
   * \brief Look up a fuzzy set by name.
   * \param setName Fuzzy-set name.
   * \return Pointer to the set, or nullptr if it does not exist.
   */
  FuzzySet* GetFuzzySet(const std::string& setName);

  /**
   * \brief Return the variable name.
   */
  std::string GetName() const { return m_name; }

  /**
   * \brief Return all fuzzy sets.
   */
  const std::vector<FuzzySet>& GetAllFuzzySets() const { return m_fuzzySets; }

  /**
   * \brief Reset every membership degree to zero.
   */
  void ResetMemberships();

private:
  std::string            m_name;           ///< Variable name.
  double                 m_minValue;       ///< Minimum value.
  double                 m_maxValue;       ///< Maximum value.
  FuzzyVariableType      m_type;           ///< Variable type.
  std::vector<FuzzySet>  m_fuzzySets;      ///< Fuzzy sets.
};

/**
 * \brief Fuzzy rule.
 *
 * An IF-THEN rule, for example:
 * IF avgEnergy is LOW AND avgSpeed is HIGH THEN energyWeight is VERY_HIGH.
 */
class FuzzyRule
{
public:
  /**
   * \brief Constructor.
   */
  FuzzyRule();

  /**
   * \brief Add an antecedent (IF part).
   * \param varName Input variable name.
   * \param setName Fuzzy-set name.
   */
  void AddAntecedent(const std::string& varName, const std::string& setName);

  /**
   * \brief Add a consequent (THEN part).
   * \param varName Output variable name.
   * \param setName Fuzzy-set name.
   */
  void AddConsequent(const std::string& varName, const std::string& setName);

  /**
   * \brief Compute the rule firing strength using the minimum operator.
   * \param inputVars Input variables by name.
   * \return Firing strength in [0, 1].
   */
  double CalculateFiringStrength(const std::map<std::string, FuzzyVariable>& inputVars) const;

  /**
   * \brief Return the consequents.
   */
  const std::map<std::string, std::string>& GetConsequents() const { return m_consequents; }

private:
  std::map<std::string, std::string> m_antecedents;  ///< Antecedents: variable name -> fuzzy-set name.
  std::map<std::string, std::string> m_consequents;  ///< Consequents: variable name -> fuzzy-set name.
};

/**
 * \brief Mamdani fuzzy inference engine.
 *
 * Implements the full Mamdani pipeline:
 * 1. fuzzification;
 * 2. rule evaluation and aggregation;
 * 3. defuzzification.
 */
class MamdaniFuzzyEngine
{
public:
  /**
   * \brief Constructor.
   */
  MamdaniFuzzyEngine();

  /**
   * \brief Add an input variable.
   * \param var Fuzzy input variable.
   */
  void AddInputVariable(const FuzzyVariable& var);

  /**
   * \brief Add an output variable.
   * \param var Fuzzy output variable.
   */
  void AddOutputVariable(const FuzzyVariable& var);

  /**
   * \brief Add a fuzzy rule.
   * \param rule Fuzzy rule.
   */
  void AddRule(const FuzzyRule& rule);

  /**
   * \brief Run fuzzy inference.
   * \param inputs Crisp inputs by variable name.
   * \return Crisp outputs by variable name.
   */
  std::map<std::string, double> Infer(const std::map<std::string, double>& inputs);

  /**
   * \brief Remove all rules.
   */
  void ClearRules();

  /**
   * \brief Return an input variable.
   */
  FuzzyVariable* GetInputVariable(const std::string& name);

  /**
   * \brief Return an output variable.
   */
  FuzzyVariable* GetOutputVariable(const std::string& name);

private:
  std::map<std::string, FuzzyVariable> m_inputVariables;   ///< Input variables.
  std::map<std::string, FuzzyVariable> m_outputVariables;  ///< Output variables.
  std::vector<FuzzyRule> m_rules;                          ///< Rule base.

  /**
   * \brief Fuzzify all inputs.
   */
  void FuzzifyInputs(const std::map<std::string, double>& inputs);

  /**
   * \brief Evaluate all rules and aggregate the outputs.
   */
  void EvaluateRules();

  /**
   * \brief Defuzzify all outputs.
   */
  std::map<std::string, double> DefuzzifyOutputs();
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_FUZZY_LOGIC_H */
