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
		afGoldenScan();
	}
}

void AfHillClimbing::processContinousMode()
{
	/* If we are in a paused state, we won't process the stats */
	if (pauseState_ == controls::AfPauseStatePaused)
		return;

	if (state_ == controls::AfStateScanning) {
		afGoldenScan();
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

bool AfHillClimbing::initialized() {
	if (initialized_1_ && initialized_2_)
		return true;

	if(!initialized_1_) {
		if (lensPosition_ != minVcmPosition_) {
			LOG(Af, Debug) << "Move lens to the minimum position";
			lensPosition_ = minVcmPosition_;
			return false;
		}

		LOG(Af, Debug) << "Initialize the first step" << " X1 " << currentContrast_;
		/* Initialize the first step */
		x1_lensPosition_ = lensPosition_;
		low_ = lensPosition_;
		f1_ = currentContrast_;
		initialized_1_ = true;

		return false;
	}

	if(!initialized_2_) {
		if (lensPosition_ != maxVcmPosition_) {
			LOG(Af, Debug) << "Move lens to the maximum position";
			lensPosition_ = maxVcmPosition_;
			return false;
		}

		LOG(Af, Debug) << "Initialize the second step" << " X2 " << currentContrast_;
		/* Initialize the second step */
		x2_lensPosition_ = lensPosition_;
		high_ = lensPosition_;
		f2_ = currentContrast_;
		initialized_2_ = true;
	}

	tolarence_ = (maxVcmPosition_ - minVcmPosition_) * searchTolerance_;

	x1_ = high_ - (high_ - low_) / phi_;
	x2_ = low_ + (high_ - low_) / phi_;

	return true;
}

void AfHillClimbing::afGoldenScan()
{
	if(!initialized()) {
		return;
	}

	if(abs(high_ - low_) < tolarence_)
	{
		LOG(Af, Debug) << "AF found the best focus position!";
		maxContrast_ = currentContrast_;
		state_ = controls::AfStateFocused;
		return;
	}

	if(check_ == 1) { 
		LOG(Af, Debug) << "Contrast " << currentContrast_;
		f1_ = currentContrast_;
		check_ = 0;
	} else if (check_ == 2) {
		LOG(Af, Debug) << "Contrast " << currentContrast_;
		f2_ = currentContrast_;
		check_ = 0;
	}

	if(f1_ < f2_) {
		low_ = x1_;
		x1_ = x2_;
		f1_ = f2_;
		x2_ = low_ + (high_ - low_) / phi_;
		lensPosition_ = x2_;
		check_ = 2;
		LOG(Af, Debug) << "Move lens to the x2 " << x2_;
	} else {
		high_ = x2_;
		x2_ = x1_;
		f2_ = f1_;
		x1_ = high_ - (high_ - low_) / phi_;
		lensPosition_ = x1_;
		check_ = 1;
		LOG(Af, Debug) << "Move lens to the x1 " << x1_;
	}
}

void AfHillClimbing::afReset()
{
	LOG(Af, Debug) << "Reset AF parameters";
	lensPosition_ = minVcmPosition_;
	state_ = controls::AfStateScanning;
	maxContrast_ = 0.0;
	initialized_1_ = false;
	initialized_2_ = false;

	setFramesToSkip(2);
}

bool AfHillClimbing::afIsOutOfFocus()
{
	/*
	* Calculate the variance change rate from the current contrast to the
	* maximum contrast. If the rate is higher than the threshold, it means
	* the sensor or the sharpness has changed to much for the current
	* focus position.
	*/
	if(currentContrast_ > maxContrast_) {
		maxContrast_ = currentContrast_;
		return false;
	}

	const uint32_t diff_var = std::abs(currentContrast_ - maxContrast_);
	const double var_ratio = diff_var / maxContrast_;
	LOG(Af, Debug) << "Current contrast: " << currentContrast_
		    << ", Max contrast: " << maxContrast_ 
			<< "Variance change rate: " << var_ratio
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
