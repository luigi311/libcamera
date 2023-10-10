/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 * Copyright (C) 2022, Theobroma Systems
 *
 * af_hill_climbing.cpp - AF Hill Climbing common algorithm
 */

#include "af_hill_climbing.h"

#include "libcamera/internal/yaml_parser.h"

/**
 * \file af_hill_climbing.h
 * \brief AF Hill Climbing common algorithm
 */

namespace libcamera::ipa::common::algorithms {

LOG_DEFINE_CATEGORY(Af)

/**
 * \class AfHillClimbing
 * \brief The base class implementing hill climbing AF control algorithm
 * \tparam Module The IPA module type for this class of algorithms
 *
 * Control part of auto focus algorithm. It calculates the lens position basing
 * on contrast measure supplied by the higher level. This way it is independent
 * from the platform.
 *
 * Derived class should call processAutofocus() for each measured contrast value
 * and set the lens to the calculated position.
 */

/**
 * \brief Initialize the Algorithm with tuning data
 * \param[in] tuningData The tuning data for the algorithm
 *
 * This function should be called in the init() function of the derived class.
 * See alse: libcamera::ipa::Algorithm::init()
 *
 * \return 0 if successful, an error code otherwise
 */
int AfHillClimbing::initBase(const YamlObject &tuningData)
{
	minVcmPosition_ = tuningData["min-vcm-position"].get<uint32_t>(0);
	maxVcmPosition_ = tuningData["max-vcm-position"].get<uint32_t>(100);
	coarseSearchStep_ = tuningData["coarse-search-step"].get<uint32_t>(30);
	fineSearchStep_ = tuningData["fine-search-step"].get<uint32_t>(1);
	fineRange_ = tuningData["fine-scan-range"].get<double>(0.05);
	maxChange_ = tuningData["max-variance-change"].get<double>(0.5);

	LOG(Af, Debug) << "minVcmPosition_: " << minVcmPosition_
		       << ", maxVcmPosition_: " << maxVcmPosition_
		       << ", coarseSearchStep_: " << coarseSearchStep_
		       << ", fineSearchStep_: " << fineSearchStep_
		       << ", fineRange_: " << fineRange_
		       << ", maxChange_: " << maxChange_;

	setMode(controls::AfModeContinuous);

	return 0;
}

/**
 * \brief Provide control values to the algorithm
 * \param[in] frame The frame number to apply the control values
 * \param[in] controls The list of user controls
 *
 * This function should be called in the queueRequest() function of the derived class.
 * See alse: libcamera::ipa::Algorithm::queueRequest()
 */
void AfHillClimbing::queueRequestBase([[maybe_unused]] const uint32_t frame, const ControlList &controls)
{
	for (auto const &[id, value] : controls) {
		switch (id) {
		case controls::AF_MODE: {
			setMode(static_cast<controls::AfModeEnum>(value.get<int32_t>()));
			break;
		}
		case controls::AF_TRIGGER: {
			setTrigger(static_cast<controls::AfTriggerEnum>(value.get<int32_t>()));
			break;
		}
		case controls::AF_PAUSE: {
			setPause(static_cast<controls::AfPauseEnum>(value.get<int32_t>()));
			break;
		}
		case controls::LENS_POSITION: {
			setLensPosition(value.get<float>());
			break;
		}
		default:
			break;
		}
	}
}

/**
 * \brief Run the auto focus algorithm loop
 * \param[in] currentContrast New value of contrast measured for current frame
 *
 * This method should be called for each new contrast value that was measured,
 * usually in the process() method.
 *
 * \return New lens position calculated by AF algorithm
 */
uint32_t AfHillClimbing::processAutofocus(double currentContrast)
{
	currentContrast_ = currentContrast;

	if (shouldSkipFrame())
		return lensPosition_;

	switch (mode_) {
	case controls::AfModeManual:
		/* Nothing to process. */
		break;
	case controls::AfModeAuto:
		processAutoMode();
		break;
	case controls::AfModeContinuous:
		processContinousMode();
		break;
	default:
		break;
	}

	return lensPosition_;
}

void AfHillClimbing::processAutoMode()
{
	if (state_ == controls::AfStateScanning) {
		afSearch();
	}
}

void AfHillClimbing::processContinousMode()
{
	/* If we are in a paused state, we won't process the stats */
	if (pauseState_ == controls::AfPauseStatePaused)
		return;

	if (state_ == controls::AfStateScanning) {
		afSearch();
		return;
	}

	/* We can re-start the scan at any moment in AfModeContinuous */
	if (afIsOutOfFocus()) {
		afReset();
	}
}

/**
 * \brief Request AF to skip n frames
 * \param[in] n Number of frames to be skipped
 *
 * Requested number of frames will not be used for AF calculation.
 */
void AfHillClimbing::setFramesToSkip(uint32_t n)
{
	if (n > framesToSkip_)
		framesToSkip_ = n;
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setMode
 */
void AfHillClimbing::setMode(controls::AfModeEnum mode)
{
	if (mode == mode_)
		return;

	LOG(Af, Debug) << "Switched AF mode from " << mode_ << " to " << mode;
	mode_ = mode;

	state_ = controls::AfStateIdle;
	pauseState_ = controls::AfPauseStateRunning;

	if (mode_ == controls::AfModeContinuous)
		afReset();
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setRange
 */
void AfHillClimbing::setRange([[maybe_unused]] controls::AfRangeEnum range)
{
	LOG(Af, Error) << __FUNCTION__ << " not implemented!";
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setSpeed
 */
void AfHillClimbing::setSpeed([[maybe_unused]] controls::AfSpeedEnum speed)
{
	LOG(Af, Error) << __FUNCTION__ << " not implemented!";
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setTrigger
 */
void AfHillClimbing::setTrigger(controls::AfTriggerEnum trigger)
{
	if (mode_ != controls::AfModeAuto) {
		LOG(Af, Warning) << __FUNCTION__ << " not possible in mode " << mode_;
		return;
	}

	LOG(Af, Debug) << "Trigger called with " << trigger;

	if (trigger == controls::AfTriggerStart)
		afReset();
	else
		state_ = controls::AfStateIdle;
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setPause
 */
void AfHillClimbing::setPause(controls::AfPauseEnum pause)
{
	if (mode_ != controls::AfModeContinuous) {
		LOG(Af, Warning) << __FUNCTION__ << " not possible in mode " << mode_;
		return;
	}

	switch (pause) {
	case controls::AfPauseImmediate:
		pauseState_ = controls::AfPauseStatePaused;
		break;
	case controls::AfPauseDeferred:
		/* \todo: add the AfPauseDeferred mode */
		LOG(Af, Warning) << "AfPauseDeferred is not supported!";
		break;
	case controls::AfPauseResume:
		pauseState_ = controls::AfPauseStateRunning;
		break;
	default:
		break;
	}
}

/**
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setLensPosition
 */
void AfHillClimbing::setLensPosition(float lensPosition)
{
	if (mode_ != controls::AfModeManual) {
		LOG(Af, Warning) << __FUNCTION__ << " not possible in mode " << mode_;
		return;
	}

	lensPosition_ = static_cast<uint32_t>(lensPosition);

	LOG(Af, Debug) << "Requesting lens position " << lensPosition_;
}

/**
 * \fn AfHillClimbing::setMeteringMode()
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setMeteringMode
 */

/**
 * \fn AfHillClimbing::setWindows()
 * \copydoc libcamera::ipa::common::algorithms::AfInterface::setWindows
 */

uint32_t AfHillClimbing::calculateAdaptiveStep(uint32_t baseStep)
{
    double rate_of_change = std::abs(currentContrast_ - previousContrast_);
    double adaptive_step_multiplier = std::max(1.0, rate_of_change / 50.0); // Adjust the divisor based on the desired sensitivity
    return static_cast<uint32_t>(std::round(baseStep * adaptive_step_multiplier));
}

void AfHillClimbing::afCoarseScan()
{
    if (coarseCompleted_)
        return;

    adaptiveStep_ = calculateAdaptiveStep(coarseSearchStep_);

    if (afScan(adaptiveStep_)) {
        coarseCompleted_ = true;
        maxContrast_ = 0;
        lensPosition_ = lensPosition_ - (lensPosition_ * fineRange_);
        previousContrast_ = 0;
        maxStep_ = std::clamp(lensPosition_ + static_cast<uint32_t>((lensPosition_ * fineRange_)),
                              0U, maxVcmPosition_);
    }
}

void AfHillClimbing::afSearch()
{
    double tolerance = (maxVcmPosition_ - minVcmPosition_) * searchTolerance_;
    if (goldenSectionSearch(minVcmPosition_, maxVcmPosition_, tolerance)) {
        LOG(Af, Debug) << "AF found the best focus position!";
        state_ = controls::AfStateFocused;
    }
}

void AfHillClimbing::afFineScan()
{
    if (!coarseCompleted_)
        return;

    adaptiveStep_ = calculateAdaptiveStep(fineSearchStep_);

    if (afScan(adaptiveStep_)) {
        LOG(Af, Debug) << "AF found the best focus position!";
        state_ = controls::AfStateFocused;
        fineCompleted_ = true;
    }
}

bool AfHillClimbing::goldenSectionSearch(uint32_t low, uint32_t high, double tolerance)
{
	uint32_t x1, x2;
    double f1, f2;

    x1 = high - (high - low) / phi_;
    x2 = low + (high - low) / phi_;

	lensPosition_ = x1;
    f1 = currentContrast_;
	
	lensPosition_ = x2;
	f2 = currentContrast_;
	
    while (abs(high - low) > tolerance) {
        if (f1 < f2) {
            low = x1;
            x1 = x2;
            f1 = f2;
            x2 = low + (high - low) / phi_;
            
			lensPosition_ = x2;
			f2 = currentContrast_;
        } else {
            high = x2;
            x2 = x1;
            f2 = f1;
            x1 = high - (high - low) / phi_;
            lensPosition_ = x1;
			f1 = currentContrast_;
        }
    }

    bestPosition_ = (f1 >= f2) ? x1 : x2;
    lensPosition_ = bestPosition_;
    currentContrast_ = (f1 >= f2) ? f1 : f2;

    return true;
}

bool AfHillClimbing::afScan(uint32_t minSteps)
{
	if (lensPosition_ + minSteps > maxStep_) {
		/* If the max step is reached, move lens to the position. */
		lensPosition_ = bestPosition_;
		return true;
	} else {
		/*
		* Find the maximum of the variance by estimating its
		* derivative. If the direction changes, it means we have passed
		* a maximum one step before.
		*/
		if ((currentContrast_ - maxContrast_) >= -(maxContrast_ * 0.1)) {
			/*
			* Positive and zero derivative:
			* The variance is still increasing. The focus could be
			* increased for the next comparison. Also, the max
			* variance and previous focus value are updated.
			*/
			bestPosition_ = lensPosition_;
			lensPosition_ += minSteps;
			maxContrast_ = currentContrast_;
		} else {
			/*
			* Negative derivative:
			* The variance starts to decrease which means the maximum
			* variance is found. Set focus step to previous good one
			* then return immediately.
			*/
			lensPosition_ = bestPosition_;
			return true;
		}
	}

	previousContrast_ = currentContrast_;
	LOG(Af, Debug) << "Previous step is " << bestPosition_
		       << ", Current step is " << lensPosition_;
	return false;
}

void AfHillClimbing::afReset()
{
	LOG(Af, Debug) << "Reset AF parameters";
	lensPosition_ = minVcmPosition_;
	maxStep_ = maxVcmPosition_;
	state_ = controls::AfStateScanning;
	previousContrast_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxContrast_ = 0.0;
	setFramesToSkip(1);
}

bool AfHillClimbing::afIsOutOfFocus()
{
	const uint32_t diff_var = std::abs(currentContrast_ - maxContrast_);
	const double var_ratio = diff_var / maxContrast_;
	LOG(Af, Debug) << "Variance change rate: " << var_ratio
		       << ", Current VCM step: " << lensPosition_;
	if (var_ratio > maxChange_)
		return true;
	else
		return false;
}

bool AfHillClimbing::shouldSkipFrame()
{
	if (framesToSkip_ > 0) {
		framesToSkip_--;
		return true;
	}

	return false;
}

} /* namespace libcamera::ipa::common::algorithms */
