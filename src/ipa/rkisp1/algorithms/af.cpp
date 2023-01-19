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

namespace {

constexpr rkisp1_cif_isp_window rectangleToIspWindow(const Rectangle &rectangle)
{
	return rkisp1_cif_isp_window{
		.h_offs = static_cast<uint16_t>(rectangle.x),
		.v_offs = static_cast<uint16_t>(rectangle.y),
		.h_size = static_cast<uint16_t>(rectangle.width),
		.v_size = static_cast<uint16_t>(rectangle.height)
	};
}

} /* namespace */

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Af::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	waitFramesLens_ = tuningData["wait-frames-lens"].get<uint32_t>(1);
	ispThreshold_ = tuningData["isp-threshold"].get<uint32_t>(128);
	ispVarShift_ = tuningData["isp-var-shift"].get<uint32_t>(4);

	LOG(RkISP1Af, Debug) << "waitFramesLens_: " << waitFramesLens_
			     << ", ispThreshold_: " << ispThreshold_
			     << ", ispVarShift_: " << ispVarShift_;

	return initBase(tuningData);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Af::configure([[maybe_unused]] IPAContext &context,
		  const IPACameraSensorInfo &configInfo)
{
	/* Default AF window of 3/4 size of the screen placed at the center */
	defaultWindow_ = Rectangle(configInfo.outputSize.width / 8,
				   configInfo.outputSize.height / 8,
				   3 * configInfo.outputSize.width / 4,
				   3 * configInfo.outputSize.height / 4);
	updateCurrentWindow(defaultWindow_);

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

	for (auto const &[id, value] : controls) {
		switch (id) {
		case controls::AF_METERING: {
			setMeteringMode(static_cast<controls::AfMeteringEnum>(value.get<int32_t>()));
			break;
		}
		case controls::AF_WINDOWS: {
			setWindows(value.get<Span<const Rectangle>>());
			break;
		}
		default:
			break;
		}
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 rkisp1_params_cfg *params)
{
	if (updateWindow_) {
		params->meas.afc_config.num_afm_win = 1;
		params->meas.afc_config.thres = ispThreshold_;
		params->meas.afc_config.var_shift = ispVarShift_;
		params->meas.afc_config.afm_win[0] = rectangleToIspWindow(*updateWindow_);

		params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AFC;
		params->module_ens |= RKISP1_CIF_ISP_MODULE_AFC;
		params->module_en_update |= RKISP1_CIF_ISP_MODULE_AFC;

		updateWindow_.reset();

		/* Wait one frame for the ISP to apply changes */
		setFramesToSkip(1);
	}
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
	if (metering == meteringMode_)
		return;

	if (metering == controls::AfMeteringWindows) {
		updateCurrentWindow(userWindow_);
	} else {
		updateCurrentWindow(defaultWindow_);
	}

	meteringMode_ = metering;
}

void Af::setWindows(Span<const Rectangle> windows)
{
	if (windows.size() != 1) {
		LOG(RkISP1Af, Error) << "Only one AF window is supported";
		return;
	}

	/* \todo Check if window size is valid for ISP */

	LOG(RkISP1Af, Debug) << "setWindows: " << userWindow_;

	userWindow_ = windows[0];

	if (meteringMode_ == controls::AfMeteringWindows)
		updateCurrentWindow(userWindow_);
}

void Af::updateCurrentWindow(const Rectangle &window)
{
	updateWindow_ = window;
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace libcamera::ipa::rkisp1::algorithms */
