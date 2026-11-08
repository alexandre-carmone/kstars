/*
    SPDX-FileCopyrightText: 2025 cfuture81

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#pragma once

#include <QJsonObject>
#include <QString>
#include <QVariantMap>

#include <cstdint>

namespace Ekos
{

/**
 * @brief Tilt plate correction maths and settings, shared by the Tilt Correction Advisory widget and the headless
 * Aberration Inspector result sent to EkosLive clients.
 *
 * Given the sensor tilt measured by the Aberration Inspector (left-to-right and top-to-bottom, in microns across
 * the sensor spans), compute() returns the adjustment of each screw of the user's tilt plate:
 *   - 3-point plates (120° spacing, 2 top + 1 bottom): Wanderer ETA, Octopi, manual 3-point plates
 *   - 4-point plates (90° spacing, push/pull): manual plates with top/bottom/left/right screws
 * Adjustments are in mm (positive = push the screw out), plus the equivalent number of screw turns.
 *
 * Plate type and mode values match the kstars.kcfg tiltPlateType / tiltMode options.
 */
namespace TiltCorrectionCalc
{

constexpr int PLATE_3POINT = 0;     // 2 top, 1 bottom (ETA, Octopi, manual 3-point)
constexpr int PLATE_4POINT = 1;     // Top/Bottom/Left/Right push-pull
constexpr int MODE_RELATIVE = 0;    // Signed deltas (apply on top of current position)
constexpr int MODE_PUSH_ONLY = 1;   // All values normalized to positive (push from zero)
constexpr int MAX_POINTS = 4;

// Wanderer ETA M54 geometry; applied automatically when the device is connected (see applyEtaOverride)
constexpr double ETA_RADIUS_MM = 64.0;
constexpr double ETA_PITCH_MM = 0.50;
// Adjustments below this are reported as "no change"
constexpr double NO_CHANGE_MM = 0.001;
// Valid ranges of the plate settings (the ranges of the Tilt Correction Advisory controls)
constexpr double MIN_RADIUS_MM = 5.0;
constexpr double MAX_RADIUS_MM = 100.0;
constexpr double MIN_PITCH_MM = 0.1;
constexpr double MAX_PITCH_MM = 2.0;
constexpr double MAX_PLATE_ANGLE_DEG = 180.0;
constexpr int THREAD_PRESET_COUNT = 10;   // entries of the thread preset combo, the last one is "Custom"

/** @brief INDI device name of the Wanderer ETA M54 tilt plate. */
QString etaDeviceName();

/** @brief Tilt plate settings, persisted per optical train with the Options::tilt* values as defaults. */
struct PlateSettings
{
    int plateType { PLATE_3POINT };
    double radiusMm { ETA_RADIUS_MM };   // bolt circle radius
    int threadIndex { 8 };               // thread preset combo index (8 = Wanderer ETA)
    double screwPitchMm { ETA_PITCH_MM };
    double plateAngleDeg { 0.0 };        // rotation of the screw ring relative to the sensor, clockwise from behind
    int mode { MODE_RELATIVE };
};

/** @brief Measured tilt, as reported by the Aberration Inspector. */
struct Inputs
{
    double lrMicrons { 0.0 };          // left-to-right tilt (positive = left higher than right)
    double tbMicrons { 0.0 };          // top-to-bottom tilt (positive = top higher than bottom)
    double spanWidthMicrons { 0.0 };   // sensor width span used for the tilt calculation
    double spanHeightMicrons { 0.0 };  // sensor height span used for the tilt calculation
};

/** @brief Adjustment of each screw. Arrays hold count entries, in the plate's point numbering (1-based on screen). */
struct Result
{
    int count { 3 };
    double adjustmentMm[MAX_POINTS] { 0.0, 0.0, 0.0, 0.0 };
    double turns[MAX_POINTS] { 0.0, 0.0, 0.0, 0.0 };
    double angleRad[MAX_POINTS] { 0.0, 0.0, 0.0, 0.0 };   // screw angle including the plate rotation (front view)
    int positionIndex[MAX_POINTS] { 0, 0, 0, 0 };         // see positionIndex()
    double maxAbsMm { 0.0 };
    double totalTiltMicrons { 0.0 };
};

/** @brief Compute the screw adjustments that cancel the measured tilt. */
Result compute(const Inputs &inputs, const PlateSettings &settings);

/**
 * @brief When a Wanderer ETA is connected, a 3-point plate takes the ETA's fixed radius and pitch (what the
 * Tilt Correction Advisory does when it detects the device; used by the headless result).
 */
PlateSettings applyEtaOverride(PlateSettings settings, bool etaConnected);

/**
 * @brief 8-way position of a screw as seen from behind the camera: 0 = right, 1 = upper right, 2 = top,
 * 3 = upper left, 4 = left, 5 = lower left, 6 = bottom, 7 = lower right.
 * @param angleRad screw angle in the front view, counter-clockwise from the right
 */
int positionIndex(double angleRad);
/** @brief Machine-readable position: right | upperRight | top | upperLeft | left | lowerLeft | bottom | lowerRight. */
QString positionKey(int index);
/** @brief Translated position word for the advisory labels. */
QString positionWord(int index);

/** @brief Settings from the per-train map (keys plateType, radius, threadIndex, screwPitch, plateAngle, mode). */
PlateSettings fromTrainMap(const QVariantMap &map, const PlateSettings &fallback);
/** @brief The per-train map of the settings, with the same keys fromTrainMap() reads. */
QVariantMap toTrainMap(const PlateSettings &settings);
/** @brief The global Options::tilt* values. */
PlateSettings globalDefaults();
/** @brief Settings of an optical train (0 = global defaults). Keys missing from the train fall back to the globals. */
PlateSettings load(uint32_t opticalTrainID);
/** @brief Persist settings to the global Options and, when the train is known, to the train. */
void save(uint32_t opticalTrainID, const PlateSettings &settings);

/** @brief Outcome of applySetting(). */
enum SettingResult { SETTING_UNKNOWN_KEY, SETTING_INVALID, SETTING_APPLIED };

/**
 * @brief Apply one EkosLive settings key to the plate settings, validating the value like the advisory's controls.
 * Keys: tiltPlateType, tiltMode, tiltRadius, tiltScrewPitch, tiltThreadIndex, tiltRotation (plate angle).
 * @param etaConnected when a Wanderer ETA is connected and the plate is 3-point, radius, pitch and thread preset
 * are fixed by the hardware and rejected.
 */
SettingResult applySetting(PlateSettings &settings, const QString &key, const QVariant &value, bool etaConnected);

/** @brief JSON representation for EkosLive (camelCase keys, machine-readable codes). */
QJsonObject toJson(const Result &result, const PlateSettings &settings, bool platePresent, bool etaConnected);

}
}
