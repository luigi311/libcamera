/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Theobroma Systems
 *
 * af_interface.h - Autofocus control algorithm interface
 */
#pragma once

#include <libcamera/control_ids.h>

namespace libcamera::ipa::common::algorithms {

class AfInterface
{
public:
	AfInterface() = default;

	virtual ~AfInterface() {}

	virtual void setMode(controls::AfModeEnum mode) = 0;

	virtual void setRange(controls::AfRangeEnum range) = 0;

	virtual void setSpeed(controls::AfSpeedEnum speed) = 0;

	virtual void setMeteringMode(controls::AfMeteringEnum metering) = 0;

	virtual void setWindows(Span<const Rectangle> windows) = 0;

	virtual void setTrigger(controls::AfTriggerEnum trigger) = 0;

	virtual void setPause(controls::AfPauseEnum pause) = 0;

	virtual void setLensPosition(float lensPosition) = 0;

	virtual controls::AfStateEnum getState() = 0;

	virtual controls::AfPauseStateEnum getPauseState() = 0;
};

} /* namespace libcamera::ipa::common::algorithms */
