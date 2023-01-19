/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Theobroma Systems
 *
 * af.cpp - RkISP1 AF hill climbing based control algorithm
 */

#include "af.h"

/**
 * \file af.h
 */

namespace libcamera::ipa::rkisp1::algorithms {

/**
 * \class Af
 * \brief AF control algorithm
 */

LOG_DEFINE_CATEGORY(RkISP1Af)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Af::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	waitFramesLens_ = tuningData["wait-frames-lens"].get<uint32_t>(1);

	LOG(RkISP1Af, Debug) << "waitFramesLens_: " << waitFramesLens_;

	return initBase(tuningData);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Af::configure([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Af::queueRequest([[maybe_unused]] IPAContext &context,
		      const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	queueRequestBase(frame, controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] rkisp1_params_cfg *params)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Af::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const rkisp1_stat_buffer *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	uint32_t sharpness = stats->params.af.window[0].sum;
	uint32_t luminance = stats->params.af.window[0].lum;

	LOG(RkISP1Af, Debug) << "lensPosition: " << context.activeState.af.lensPosition
			     << ", Sharpness: " << sharpness
			     << ", Luminance: " << luminance;

	uint32_t lensPosition = processAutofocus(sharpness);

	if (lensPosition != context.activeState.af.lensPosition) {
		context.activeState.af.lensPosition = lensPosition;
		context.activeState.af.applyLensCtrls = true;
		setFramesToSkip(waitFramesLens_);
	}
}

void Af::setMeteringMode([[maybe_unused]] controls::AfMeteringEnum metering)
{
	LOG(RkISP1Af, Error) << __FUNCTION__ << " not implemented!";
}

void Af::setWindows([[maybe_unused]] Span<const Rectangle> windows)
{
	LOG(RkISP1Af, Error) << __FUNCTION__ << " not implemented!";
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace libcamera::ipa::rkisp1::algorithms */
