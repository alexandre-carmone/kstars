/*
    SPDX-FileCopyrightText: 2025 cfuture81

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "tiltcorrectioncalc.h"

#include "Options.h"
#include "ekos/auxiliary/opticaltrainsettings.h"

#include <KLocalizedString>

#include <QJsonArray>

#include <algorithm>
#include <cmath>

namespace Ekos
{
namespace TiltCorrectionCalc
{

QString etaDeviceName()
{
    return QStringLiteral("Wanderer ETA M54");
}

Result compute(const Inputs &inputs, const PlateSettings &settings)
{
    Result result;
    result.count = settings.plateType == PLATE_4POINT ? 4 : 3;

    // The tilt plane to CORRECT is the opposite of the measured tilt.
    // LR slope: microns per micron across sensor width
    // TB slope: microns per micron across sensor height
    const double lrSlope = (inputs.spanWidthMicrons > 0) ? inputs.lrMicrons / inputs.spanWidthMicrons : 0.0;
    const double tbSlope = (inputs.spanHeightMicrons > 0) ? inputs.tbMicrons / inputs.spanHeightMicrons : 0.0;

    // Convert radius from mm to microns for consistent units
    const double R = settings.radiusMm * 1000.0;

    // Tilt-plate rotation: the screw ring may sit at a fixed angular offset relative to the sensor (e.g. a 3-point
    // plate bolted at 15°). The measured tilt plane is fixed in sensor coordinates, so each screw's required piston
    // is evaluated at its rotated position. Positive = clockwise viewed from behind.
    const double phi = settings.plateAngleDeg * M_PI / 180.0;

    // Screw angles in the front view (telescope side):
    // 3-point: Point 1 at 150° (upper right in rear view), Point 2 at 270° (bottom), Point 3 at 30° (upper left)
    // 4-point: Screw 1 at 45° (top left in rear view), Screw 2 at 135° (top right), Screw 3 at 315° (bottom left),
    //          Screw 4 at 225° (bottom right)
    const double threePoint[3] = { 150.0, 270.0, 30.0 };
    const double fourPoint[4] = { 45.0, 135.0, 315.0, 225.0 };
    const double *baseAngles = result.count == 4 ? fourPoint : threePoint;

    // For each point, the correction needed is the NEGATIVE of the tilt at that position
    // Z_correction = -(lrSlope * x + tbSlope * y)
    // These are RELATIVE deltas — apply on top of current position, can be + or -.
    for (int i = 0; i < result.count; i++)
    {
        result.angleRad[i] = baseAngles[i] * M_PI / 180.0 + phi;
        const double x = R * std::cos(result.angleRad[i]);
        const double y = R * std::sin(result.angleRad[i]);
        // microns to mm
        result.adjustmentMm[i] = -(lrSlope * x + tbSlope * y) / 1000.0;
        result.positionIndex[i] = positionIndex(result.angleRad[i]);
    }

    // Apply push-only normalization if requested: shift values so minimum is 0
    if (settings.mode == MODE_PUSH_ONLY)
    {
        const double minAdj = *std::min_element(result.adjustmentMm, result.adjustmentMm + result.count);
        for (int i = 0; i < result.count; i++)
            result.adjustmentMm[i] -= minAdj;
    }

    for (int i = 0; i < result.count; i++)
    {
        result.turns[i] = settings.screwPitchMm != 0.0 ? result.adjustmentMm[i] / settings.screwPitchMm : 0.0;
        result.maxAbsMm = std::max(result.maxAbsMm, std::abs(result.adjustmentMm[i]));
    }
    result.totalTiltMicrons = std::hypot(inputs.lrMicrons, inputs.tbMicrons);
    return result;
}

PlateSettings applyEtaOverride(PlateSettings settings, bool etaConnected)
{
    if (etaConnected && settings.plateType == PLATE_3POINT)
    {
        settings.radiusMm = ETA_RADIUS_MM;
        settings.screwPitchMm = ETA_PITCH_MM;
    }
    return settings;
}

// Map a screw's math angle (radians, incl. plate rotation) to one of the 8 positions of the rear-view diagram
// (0 = right, 2 = top, 4 = left, 6 = bottom). In that view the visual axes are: right = -cos(angle),
// up = sin(angle) (left-right mirrored, screen-Y is down).
int positionIndex(double angleRad)
{
    const double right = -std::cos(angleRad);
    const double up = std::sin(angleRad);
    double deg = std::atan2(up, right) * 180.0 / M_PI;  // 0 = right, 90 = top
    if (deg < 0)
        deg += 360.0;
    return static_cast<int>(std::lround(deg / 45.0)) % 8;
}

QString positionKey(int index)
{
    static const char *const keys[8] = { "right", "upperRight", "top", "upperLeft", "left", "lowerLeft", "bottom", "lowerRight" };
    if (index < 0 || index >= 8)
        return QString();
    return QString::fromLatin1(keys[index]);
}

QString positionWord(int index)
{
    switch (index)
    {
        case 0:
            return i18nc("screw position", "right");
        case 1:
            return i18nc("screw position", "upper-right");
        case 2:
            return i18nc("screw position", "top");
        case 3:
            return i18nc("screw position", "upper-left");
        case 4:
            return i18nc("screw position", "left");
        case 5:
            return i18nc("screw position", "lower-left");
        case 6:
            return i18nc("screw position", "bottom");
        default:
            return i18nc("screw position", "lower-right");
    }
}

PlateSettings fromTrainMap(const QVariantMap &map, const PlateSettings &fallback)
{
    auto asInt = [&map](const QString & key, int defaultValue) -> int
    {
        return map.contains(key) ? map.value(key).toInt() : defaultValue;
    };
    auto asDouble = [&map](const QString & key, double defaultValue) -> double
    {
        return map.contains(key) ? map.value(key).toDouble() : defaultValue;
    };

    PlateSettings settings;
    settings.plateType = asInt(QStringLiteral("plateType"), fallback.plateType);
    settings.radiusMm = asDouble(QStringLiteral("radius"), fallback.radiusMm);
    settings.threadIndex = asInt(QStringLiteral("threadIndex"), fallback.threadIndex);
    settings.screwPitchMm = asDouble(QStringLiteral("screwPitch"), fallback.screwPitchMm);
    settings.plateAngleDeg = asDouble(QStringLiteral("plateAngle"), fallback.plateAngleDeg);
    settings.mode = asInt(QStringLiteral("mode"), fallback.mode);
    return settings;
}

QVariantMap toTrainMap(const PlateSettings &settings)
{
    QVariantMap map;
    map[QStringLiteral("plateType")]   = settings.plateType;
    map[QStringLiteral("radius")]      = settings.radiusMm;
    map[QStringLiteral("threadIndex")] = settings.threadIndex;
    map[QStringLiteral("screwPitch")]  = settings.screwPitchMm;
    map[QStringLiteral("plateAngle")]  = settings.plateAngleDeg;
    map[QStringLiteral("mode")]        = settings.mode;
    return map;
}

PlateSettings globalDefaults()
{
    PlateSettings settings;
    settings.plateType = static_cast<int>(Options::tiltPlateType());
    settings.radiusMm = Options::tiltRadius();
    settings.threadIndex = static_cast<int>(Options::tiltThreadIndex());
    settings.screwPitchMm = Options::tiltScrewPitch();
    // Options::tiltRotation() holds the plate angle (repurposed from the old camera-rotation control)
    settings.plateAngleDeg = Options::tiltRotation();
    settings.mode = static_cast<int>(Options::tiltMode());
    return settings;
}

PlateSettings load(uint32_t opticalTrainID)
{
    const PlateSettings defaults = globalDefaults();
    if (opticalTrainID == 0)
        return defaults;

    // Per-train settings (if any). Missing keys fall back to the global Options values,
    // so a train that was never configured inherits the last global defaults.
    OpticalTrainSettings::Instance()->setOpticalTrainID(opticalTrainID);
    const QVariant stored = OpticalTrainSettings::Instance()->getOneSetting(OpticalTrainSettings::FocusTilt);
    if (!stored.isValid())
        return defaults;
    return fromTrainMap(stored.toJsonObject().toVariantMap(), defaults);
}

void save(uint32_t opticalTrainID, const PlateSettings &settings)
{
    // Always update the global defaults so a brand-new train inherits the latest values.
    Options::setTiltPlateType(settings.plateType);
    Options::setTiltRadius(settings.radiusMm);
    Options::setTiltThreadIndex(settings.threadIndex);
    Options::setTiltScrewPitch(settings.screwPitchMm);
    Options::setTiltRotation(settings.plateAngleDeg);  // stores the plate angle
    Options::setTiltMode(settings.mode);

    // Also persist per optical train when we know which one we're on.
    if (opticalTrainID != 0)
    {
        OpticalTrainSettings::Instance()->setOpticalTrainID(opticalTrainID);
        OpticalTrainSettings::Instance()->setOneSetting(OpticalTrainSettings::FocusTilt, toTrainMap(settings));
    }
}

QJsonObject toJson(const Result &result, const PlateSettings &settings, bool platePresent, bool etaConnected)
{
    QJsonObject json;
    json.insert(QStringLiteral("valid"), true);
    json.insert(QStringLiteral("present"), platePresent);
    json.insert(QStringLiteral("etaConnected"), etaConnected);
    json.insert(QStringLiteral("plateType"), settings.plateType);
    json.insert(QStringLiteral("mode"), settings.mode);
    json.insert(QStringLiteral("radiusMm"), settings.radiusMm);
    json.insert(QStringLiteral("screwPitchMm"), settings.screwPitchMm);
    json.insert(QStringLiteral("threadIndex"), settings.threadIndex);
    json.insert(QStringLiteral("plateAngleDeg"), settings.plateAngleDeg);
    json.insert(QStringLiteral("totalTiltMicrons"), result.totalTiltMicrons);
    json.insert(QStringLiteral("maxAdjustmentMm"), result.maxAbsMm);

    QJsonArray points;
    for (int i = 0; i < result.count; i++)
    {
        double angleDeg = std::fmod(result.angleRad[i] * 180.0 / M_PI, 360.0);
        if (angleDeg < 0)
            angleDeg += 360.0;
        QJsonObject point;
        point.insert(QStringLiteral("index"), i + 1);
        point.insert(QStringLiteral("angleDeg"), angleDeg);
        point.insert(QStringLiteral("position"), positionKey(result.positionIndex[i]));
        point.insert(QStringLiteral("adjustmentMm"), result.adjustmentMm[i]);
        point.insert(QStringLiteral("turns"), result.turns[i]);
        points.append(point);
    }
    json.insert(QStringLiteral("points"), points);
    return json;
}

}
}
