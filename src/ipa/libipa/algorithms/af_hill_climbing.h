/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 * Copyright (C) 2022, Theobroma Systems
 *
 * af_hill_climbing.h - AF Hill Climbing common algorithm
 */

#pragma once

#include <libcamera/base/log.h>

#include "af_interface.h"
#include <cmath>

namespace libcamera {

class YamlObject;

namespace ipa::common::algorithms {

LOG_DECLARE_CATEGORY(Af)

class AfHillClimbing : public AfInterface
{
public:
	AfHillClimbing() = default;
	virtual ~AfHillClimbing() {}

	controls::AfStateEnum getState() final { return state_; }
	controls::AfPauseStateEnum getPauseState() final { return pauseState_; }

	/* These methods should be implemented by the derived class: */
	virtual void setMeteringMode(controls::AfMeteringEnum metering) = 0;
	virtual void setWindows(Span<const Rectangle> windows) = 0;

protected:
	int initBase(const YamlObject &tuningData);
	void queueRequestBase(const uint32_t frame, const ControlList &controls);
	uint32_t processAutofocus(double currentContrast);
	void setFramesToSkip(uint32_t n);

private:
	void setMode(controls::AfModeEnum mode) final;
	void setRange(controls::AfRangeEnum range) final;
	void setSpeed(controls::AfSpeedEnum speed) final;
	void setTrigger(controls::AfTriggerEnum trigger) final;
	void setPause(controls::AfPauseEnum pause) final;
	void setLensPosition(float lensPosition) final;

	void processAutoMode();
	void processContinousMode();
	void afCoarseScan();
	void afFineScan();
	bool afScan(uint32_t minSteps);
	void afSearch();
	void afReset();
	bool afIsOutOfFocus();
	bool shouldSkipFrame();
	uint32_t calculateAdaptiveStep(uint32_t baseStep);
	bool goldenSectionSearch();

	controls::AfModeEnum mode_ = controls::AfModeManual;
	controls::AfStateEnum state_ = controls::AfStateIdle;
	controls::AfPauseStateEnum pauseState_ = controls::AfPauseStateRunning;
	
	const double searchTolerance_ = 0.01;
	uint32_t tolerance_ = floor((maxVcmPosition_ - minVcmPosition_) * searchTolerance_);
	/* The golden ratio. */
	const double phi_ = (1.0 + std::sqrt(5.0)) / 2.0;
	uint32_t adaptiveStep_ = 0;

	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t lensPosition_ = 0;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestPosition_ = 0;

	/* Current AF statistic contrast. */
	double currentContrast_ = 0;
	/* It is used to determine the derivative during scanning */
	double previousContrast_ = 0;
	double maxContrast_ = 0;
	/* The designated maximum range of focus scanning. */
	uint32_t maxStep_ = 0;
	/* If the coarse scan completes, it is set to true. */
	bool coarseCompleted_ = false;
	/* If the fine scan completes, it is set to true. */
	bool fineCompleted_ = false;

	uint32_t framesToSkip_ = 0;

	/*
	* Focus steps range of the VCM control
	* \todo should be obtained from the VCM driver
	*/
	uint32_t minVcmPosition_;
	uint32_t maxVcmPosition_;

	/* Minimum focus step for searching appropriate focus */
	uint32_t coarseSearchStep_;
	uint32_t fineSearchStep_;

	/* Fine scan range 0 < fineRange_ < 1 */
	double fineRange_;

	/* Max ratio of variance change, 0.0 < maxChange_ < 1.0 */
	double maxChange_;
};

} /* namespace ipa::common::algorithms */
} /* namespace libcamera */
