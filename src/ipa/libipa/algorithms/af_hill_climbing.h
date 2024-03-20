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
	bool initialized();
	void afGoldenScan();
	void afFineScan();
	bool afScan(uint32_t minSteps);
	void afReset();
	bool aboveVarianceThreshold();
	bool afIsOutOfFocus();
	bool shouldSkipFrame();

	controls::AfModeEnum mode_ = controls::AfModeManual;
	controls::AfStateEnum state_ = controls::AfStateIdle;
	controls::AfPauseStateEnum pauseState_ = controls::AfPauseStateRunning;


	uint32_t low_ = 0;
	uint32_t high_ = 0;

	bool initialized_1_ = false;
	uint32_t x1_lensPosition_ = 0;
	uint32_t x1_ = 0;
	double f1_ = 0;

	bool initialized_2_ = false;
	uint32_t x2_lensPosition_ = 0;
	uint32_t x2_ = 0;
	double f2_ = 0;

	uint32_t check_ = 0;

	const double searchTolerance_ = 0.01;
	const double phi_ = 1.618033988;

	double tolarence_;



	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t lensPosition_ = 0;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestPosition_ = 0;

	/* Current AF statistic contrast. */
	double currentContrast_ = 0;
	/* It is used to determine the derivative during scanning */
	double maxContrast_ = 0;

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
