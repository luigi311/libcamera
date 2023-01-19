/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Theobroma Systems
 *
 * af.h - RkISP1 AF hill climbing based control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include "libipa/algorithms/af_hill_climbing.h"

#include "algorithm.h"

namespace libcamera::ipa::rkisp1::algorithms {

class Af : public ipa::common::algorithms::AfHillClimbing, public Algorithm
{
public:
	Af() = default;
	~Af() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, rkisp1_params_cfg *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, const rkisp1_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	void setMeteringMode(controls::AfMeteringEnum metering) final;
	void setWindows(Span<const Rectangle> windows) final;

	void updateCurrentWindow(const Rectangle &window);

	controls::AfMeteringEnum meteringMode_ = controls::AfMeteringAuto;
	Rectangle defaultWindow_;
	Rectangle userWindow_;
	std::optional<Rectangle> updateWindow_;
	uint32_t ispThreshold_;
	uint32_t ispVarShift_;

	/* Wait number of frames after changing lens position */
	uint32_t waitFramesLens_;
};

} /* namespace libcamera::ipa::rkisp1::algorithms */
