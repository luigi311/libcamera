/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Theobroma Systems
 *
 * af_interface.cpp - Autofocus control algorithm interface
 */

#include "af_interface.h"

/**
 * \file af_interface.h
 * \brief AF algorithm common interface
 */

namespace libcamera::ipa::common::algorithms {

/**
 * \class AfInterface
 * \brief Common interface for auto-focus algorithms
 * \tparam Module The IPA module type for this class of algorithms
 *
 * The AfInterface class defines a standard interface for IPA auto focus
 * algorithms.
 */

/**
 * \fn AfInterface::setMode()
 * \brief Set auto focus mode
 * \param[in] mode AF mode
 */

/**
 * \fn AfInterface::setRange()
 * \brief set the range of focus distances that is scanned
 * \param[in] range AF range
 */

/**
 * \fn AfInterface::setSpeed()
 * \brief Set how fast algorithm should move the lens
 * \param[in] speed Lens move speed
 */

/**
 * \fn AfInterface::setMeteringMode()
 * \brief Set AF metering mode
 * \param[in] metering AF metering mode
 */

/**
 * \fn AfInterface::setWindows()
 * \brief Set AF windows
 * \param[in] windows AF windows
 *
 * Sets the focus windows used by the AF algorithm when AfMetering is set
 * to AfMeteringWindows.
 */

/**
 * \fn AfInterface::setTrigger()
 * \brief Starts or cancels the autofocus scan
 * \param[in] trigger Trigger mode
 */

/**
 * \fn AfInterface::setPause()
 * \brief Pause the autofocus while in AfModeContinuous mode.
 * \param[in] pause Pause mode
 */

/**
 * \fn AfInterface::setLensPosition()
 * \brief Set the lens position while in AfModeManual
 * \param[in] lensPosition Lens position
 */

/**
 * \fn AfInterface::getState()
 * \brief Get the current state of the AF algorithm
 * \return AF state
 */

/**
 * \fn AfInterface::getPauseState()
 * \brief Get the current pause state of the AF algorithm.
 * \return AF pause state
 *
 * Only applicable in continuous (AfModeContinuous) mode. In other modes,
 * AfPauseStateRunning is always returned.
 */

} /* namespace libcamera::ipa::common::algorithms */
