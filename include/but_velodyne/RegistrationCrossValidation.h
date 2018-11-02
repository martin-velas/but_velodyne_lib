/*
 * RegistrationCrossValidation.h
 *
 *  Created on: Oct 31, 2018
 *      Author: Martin Velas (ivelas@fit.vutbr.cz)
 */

#ifndef REGISTRATIONCROSSVALIDATION_H_
#define REGISTRATIONCROSSVALIDATION_H_

#include <vector>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

namespace but_velodyne {

class RegistrationCrossValidation {

public:

  RegistrationCrossValidation(const std::vector<RegistrationOutcome> &nfold_reg_outcomes_);

  RegistrationOutcome findBest(void);

protected:

  bool containsConvergedResults(void) const;

  void removeNotConverged(void);

  void keepConselsus(void);

  RegistrationOutcome pickBestByError(void);

private:

  std::vector<RegistrationOutcome> nfold_reg_outcomes;
};

} /* namespace but_velodyne */

#endif /* REGISTRATIONCROSSVALIDATION_H_ */
