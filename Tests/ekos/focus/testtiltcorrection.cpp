/*
    SPDX-FileCopyrightText: 2026 Alexandre Carmone <acarmone-prestataire@ina.fr>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "ekos/focus/tiltcorrectioncalc.h"

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QtTest/QTest>
#else
#include <QTest>
#endif

#include <KLocalizedString>

#include <QJsonArray>
#include <QJsonObject>
#include <QObject>

#include <cmath>

using namespace Ekos::TiltCorrectionCalc;

// Headless tests of the tilt plate correction maths shared by the Tilt Correction Advisory and EkosLive.
// Inputs are the Aberration Inspector tilt of the testaberrationinspector fixture (14400 x 10400 um spans).
// load()/save() need the KStars user database and are covered by the manual test procedure instead.
class TestTiltCorrection : public QObject
{
        Q_OBJECT

    public:
        TestTiltCorrection() = default;
        ~TestTiltCorrection() override = default;

    private Q_SLOTS:
        void threePointRelative();
        void threePointPushOnly();
        void threePointPlateAngle90();
        void threePointPlateAngleMinus90();
        void threePointOtherGeometry();
        void fourPointRelative();
        void fourPointPushOnly();
        void fourPointTBOnly();
        void zeroTiltAndZeroSpan();
        void positionIndexTable();
        void etaOverride();
        void trainMapRoundTrip();
        void applySettingValidation();
        void applySettingLockedByEta();
        void tiltToJson();
};

#include "testtiltcorrection.moc"

namespace
{

#define COMPARE_DOUBLE(actual, expected, tolerance) \
    QVERIFY2(std::abs((actual) - (expected)) <= (tolerance), \
             qPrintable(QString("%1: actual %2, expected %3").arg(#actual).arg(actual, 0, 'f', 6).arg(expected, 0, 'f', 6)))

const double TOL = 1e-6;

Inputs lrTilt()
{
    return Inputs { -20.0, 0.0, 14400.0, 10400.0 };
}

PlateSettings threePoint()
{
    PlateSettings s;
    s.plateType = PLATE_3POINT;
    s.radiusMm = 64.0;
    s.threadIndex = 8;
    s.screwPitchMm = 0.5;
    s.plateAngleDeg = 0.0;
    s.mode = MODE_RELATIVE;
    return s;
}

// Expected adjustment of a screw at the given front-view angle for the fixture's LR tilt: with a 64 mm radius and a
// -20 um tilt over 14400 um, the slope is -20/14400 and adj = -(slope * 64000 * cos(angle)) / 1000 mm.
double lrAdjustment(double angleDeg)
{
    return -((-20.0 / 14400.0) * 64000.0 * std::cos(angleDeg * M_PI / 180.0)) / 1000.0;
}

}

void TestTiltCorrection::threePointRelative()
{
    const Result r = compute(lrTilt(), threePoint());
    QCOMPARE(r.count, 3);
    COMPARE_DOUBLE(r.adjustmentMm[0], lrAdjustment(150.0), TOL);   // -0.076980
    COMPARE_DOUBLE(r.adjustmentMm[1], 0.0, 1e-9);                  // bottom screw is on the tilt axis
    COMPARE_DOUBLE(r.adjustmentMm[2], lrAdjustment(30.0), TOL);    // +0.076980
    COMPARE_DOUBLE(r.adjustmentMm[0], -0.076980, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[2], 0.076980, 1e-6);
    COMPARE_DOUBLE(r.turns[0], -0.153960, 1e-6);
    COMPARE_DOUBLE(r.turns[1], 0.0, 1e-9);
    COMPARE_DOUBLE(r.turns[2], 0.153960, 1e-6);
    COMPARE_DOUBLE(r.maxAbsMm, 0.076980, 1e-6);
    COMPARE_DOUBLE(r.totalTiltMicrons, 20.0, 1e-9);
    QCOMPARE(r.positionIndex[0], 1);   // upper right (rear view)
    QCOMPARE(r.positionIndex[1], 6);   // bottom
    QCOMPARE(r.positionIndex[2], 3);   // upper left
    COMPARE_DOUBLE(r.angleRad[0], 150.0 * M_PI / 180.0, 1e-9);
    COMPARE_DOUBLE(r.angleRad[1], 270.0 * M_PI / 180.0, 1e-9);
    COMPARE_DOUBLE(r.angleRad[2], 30.0 * M_PI / 180.0, 1e-9);
    QCOMPARE(positionKey(r.positionIndex[0]), QStringLiteral("upperRight"));
    QCOMPARE(positionKey(r.positionIndex[1]), QStringLiteral("bottom"));
    QCOMPARE(positionKey(r.positionIndex[2]), QStringLiteral("upperLeft"));
    QCOMPARE(positionWord(1), i18nc("screw position", "upper-right"));
    QCOMPARE(positionWord(6), i18nc("screw position", "bottom"));
}

void TestTiltCorrection::threePointPushOnly()
{
    PlateSettings s = threePoint();
    s.mode = MODE_PUSH_ONLY;
    const Result r = compute(lrTilt(), s);
    QCOMPARE(r.count, 3);
    COMPARE_DOUBLE(r.adjustmentMm[0], 0.0, 1e-9);
    COMPARE_DOUBLE(r.adjustmentMm[1], 0.076980, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[2], 0.153960, 1e-6);
    COMPARE_DOUBLE(r.turns[2], 0.307920, 1e-6);
    COMPARE_DOUBLE(r.maxAbsMm, 0.153960, 1e-6);
    // Push-only does not change the screw positions
    QCOMPARE(r.positionIndex[0], 1);
    QCOMPARE(r.positionIndex[1], 6);
    QCOMPARE(r.positionIndex[2], 3);
}

void TestTiltCorrection::threePointPlateAngle90()
{
    PlateSettings s = threePoint();
    s.plateAngleDeg = 90.0;
    const Result r = compute(lrTilt(), s);
    COMPARE_DOUBLE(r.adjustmentMm[0], lrAdjustment(240.0), TOL);   // -0.044444
    COMPARE_DOUBLE(r.adjustmentMm[1], lrAdjustment(360.0), TOL);   // +0.088889
    COMPARE_DOUBLE(r.adjustmentMm[2], lrAdjustment(120.0), TOL);   // -0.044444
    COMPARE_DOUBLE(r.adjustmentMm[0], -0.044444, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[1], 0.088889, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[2], -0.044444, 1e-6);
    QCOMPARE(r.positionIndex[0], 7);   // lower right
    QCOMPARE(r.positionIndex[1], 4);   // left
    QCOMPARE(r.positionIndex[2], 1);   // upper right
}

void TestTiltCorrection::threePointPlateAngleMinus90()
{
    PlateSettings s = threePoint();
    s.plateAngleDeg = -90.0;
    const Result r = compute(lrTilt(), s);
    COMPARE_DOUBLE(r.adjustmentMm[0], lrAdjustment(60.0), TOL);    // +0.044444
    COMPARE_DOUBLE(r.adjustmentMm[1], lrAdjustment(180.0), TOL);   // -0.088889
    COMPARE_DOUBLE(r.adjustmentMm[2], lrAdjustment(-60.0), TOL);   // +0.044444
    QCOMPARE(r.positionIndex[0], 3);   // upper left
    QCOMPARE(r.positionIndex[1], 0);   // right
    QCOMPARE(r.positionIndex[2], 5);   // lower left
    // The reported angles keep the plate rotation (point 3 ends up at -60 degrees, reported as 300)
    COMPARE_DOUBLE(r.angleRad[2], -60.0 * M_PI / 180.0, 1e-9);
    const QJsonObject json = toJson(r, s, true, false);
    COMPARE_DOUBLE(json.value(QStringLiteral("points")).toArray()[2].toObject().value(QStringLiteral("angleDeg")).toDouble(),
                   300.0, 1e-9);
}

void TestTiltCorrection::threePointOtherGeometry()
{
    // A 40 mm plate with M4 screws (0.7 mm): adjustments scale with the radius, turns with the pitch
    PlateSettings s = threePoint();
    s.radiusMm = 40.0;
    s.screwPitchMm = 0.7;
    const Result r = compute(lrTilt(), s);
    const double expected = -((-20.0 / 14400.0) * 40000.0 * std::cos(150.0 * M_PI / 180.0)) / 1000.0;
    COMPARE_DOUBLE(r.adjustmentMm[0], expected, TOL);
    COMPARE_DOUBLE(r.adjustmentMm[0], -0.048113, 1e-6);
    COMPARE_DOUBLE(r.turns[0], expected / 0.7, TOL);
    COMPARE_DOUBLE(r.turns[0], -0.068733, 1e-6);
    COMPARE_DOUBLE(r.maxAbsMm, 0.048113, 1e-6);
}

void TestTiltCorrection::fourPointRelative()
{
    PlateSettings s = threePoint();
    s.plateType = PLATE_4POINT;
    const Result r = compute(lrTilt(), s);
    QCOMPARE(r.count, 4);
    COMPARE_DOUBLE(r.adjustmentMm[0], 0.062854, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[1], -0.062854, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[2], 0.062854, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[3], -0.062854, 1e-6);
    COMPARE_DOUBLE(r.turns[0], 0.125708, 1e-6);
    COMPARE_DOUBLE(r.turns[1], -0.125708, 1e-6);
    COMPARE_DOUBLE(r.maxAbsMm, 0.062854, 1e-6);
    QCOMPARE(r.positionIndex[0], 3);   // upper left
    QCOMPARE(r.positionIndex[1], 1);   // upper right
    QCOMPARE(r.positionIndex[2], 5);   // lower left
    QCOMPARE(r.positionIndex[3], 7);   // lower right
}

void TestTiltCorrection::fourPointPushOnly()
{
    PlateSettings s = threePoint();
    s.plateType = PLATE_4POINT;
    s.mode = MODE_PUSH_ONLY;
    const Result r = compute(lrTilt(), s);
    QCOMPARE(r.count, 4);
    COMPARE_DOUBLE(r.adjustmentMm[0], 0.125708, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[1], 0.0, 1e-9);
    COMPARE_DOUBLE(r.adjustmentMm[2], 0.125708, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[3], 0.0, 1e-9);
    COMPARE_DOUBLE(r.maxAbsMm, 0.125708, 1e-6);
    COMPARE_DOUBLE(r.turns[0], 0.251416, 1e-6);
}

void TestTiltCorrection::fourPointTBOnly()
{
    PlateSettings s = threePoint();
    s.plateType = PLATE_4POINT;
    const Result r = compute(Inputs { 0.0, -32.0, 14400.0, 10400.0 }, s);
    QCOMPARE(r.count, 4);
    COMPARE_DOUBLE(r.adjustmentMm[0], 0.139246, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[1], 0.139246, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[2], -0.139246, 1e-6);
    COMPARE_DOUBLE(r.adjustmentMm[3], -0.139246, 1e-6);
    COMPARE_DOUBLE(r.totalTiltMicrons, 32.0, 1e-9);
}

void TestTiltCorrection::zeroTiltAndZeroSpan()
{
    const Result zero = compute(Inputs { 0.0, 0.0, 14400.0, 10400.0 }, threePoint());
    for (int i = 0; i < zero.count; i++)
    {
        COMPARE_DOUBLE(zero.adjustmentMm[i], 0.0, 1e-12);
        COMPARE_DOUBLE(zero.turns[i], 0.0, 1e-12);
    }
    COMPARE_DOUBLE(zero.maxAbsMm, 0.0, 1e-12);
    COMPARE_DOUBLE(zero.totalTiltMicrons, 0.0, 1e-12);

    // Unknown spans must not divide by zero
    const Result noSpan = compute(Inputs { -20.0, 15.0, 0.0, 0.0 }, threePoint());
    for (int i = 0; i < noSpan.count; i++)
    {
        QVERIFY(std::isfinite(noSpan.adjustmentMm[i]));
        QVERIFY(std::isfinite(noSpan.turns[i]));
        COMPARE_DOUBLE(noSpan.adjustmentMm[i], 0.0, 1e-12);
    }
    COMPARE_DOUBLE(noSpan.totalTiltMicrons, 25.0, 1e-9);

    // A zero pitch gives no turns rather than infinity
    PlateSettings s = threePoint();
    s.screwPitchMm = 0.0;
    const Result noPitch = compute(lrTilt(), s);
    for (int i = 0; i < noPitch.count; i++)
        QVERIFY(std::isfinite(noPitch.turns[i]));
}

void TestTiltCorrection::positionIndexTable()
{
    const int expected[8] = { 4, 3, 2, 1, 0, 7, 6, 5 };
    for (int i = 0; i < 8; i++)
        QCOMPARE(positionIndex(i * 45.0 * M_PI / 180.0), expected[i]);

    QStringList keys;
    for (int i = 0; i < 8; i++)
    {
        QVERIFY(!positionKey(i).isEmpty());
        QVERIFY(!keys.contains(positionKey(i)));
        keys.append(positionKey(i));
        QVERIFY(!positionWord(i).isEmpty());
    }
    QCOMPARE(positionKey(0), QStringLiteral("right"));
    QCOMPARE(positionKey(2), QStringLiteral("top"));
    QCOMPARE(positionKey(4), QStringLiteral("left"));
    QCOMPARE(positionKey(6), QStringLiteral("bottom"));
    QVERIFY(positionKey(8).isEmpty());
    QVERIFY(positionKey(-1).isEmpty());
}

void TestTiltCorrection::etaOverride()
{
    PlateSettings s = threePoint();
    s.radiusMm = 50.0;
    s.screwPitchMm = 0.7;

    const PlateSettings overridden = applyEtaOverride(s, true);
    COMPARE_DOUBLE(overridden.radiusMm, 64.0, 1e-12);
    COMPARE_DOUBLE(overridden.screwPitchMm, 0.5, 1e-12);
    QCOMPARE(overridden.plateType, PLATE_3POINT);
    QCOMPARE(overridden.threadIndex, 8);

    const PlateSettings notConnected = applyEtaOverride(s, false);
    COMPARE_DOUBLE(notConnected.radiusMm, 50.0, 1e-12);
    COMPARE_DOUBLE(notConnected.screwPitchMm, 0.7, 1e-12);

    s.plateType = PLATE_4POINT;
    const PlateSettings fourPoint = applyEtaOverride(s, true);
    COMPARE_DOUBLE(fourPoint.radiusMm, 50.0, 1e-12);
    COMPARE_DOUBLE(fourPoint.screwPitchMm, 0.7, 1e-12);
    QCOMPARE(etaDeviceName(), QStringLiteral("Wanderer ETA M54"));
}

void TestTiltCorrection::trainMapRoundTrip()
{
    PlateSettings s;
    s.plateType = PLATE_4POINT;
    s.radiusMm = 70.5;
    s.threadIndex = 3;
    s.screwPitchMm = 0.7;
    s.plateAngleDeg = 15.0;
    s.mode = MODE_PUSH_ONLY;

    const QVariantMap map = toTrainMap(s);
    const QStringList expectedKeys = { "mode", "plateAngle", "plateType", "radius", "screwPitch", "threadIndex" };
    QStringList keys = map.keys();
    keys.sort();
    QCOMPARE(keys, expectedKeys);

    const PlateSettings fallback = threePoint();
    const PlateSettings back = fromTrainMap(map, fallback);
    QCOMPARE(back.plateType, PLATE_4POINT);
    COMPARE_DOUBLE(back.radiusMm, 70.5, 1e-12);
    QCOMPARE(back.threadIndex, 3);
    COMPARE_DOUBLE(back.screwPitchMm, 0.7, 1e-12);
    COMPARE_DOUBLE(back.plateAngleDeg, 15.0, 1e-12);
    QCOMPARE(back.mode, MODE_PUSH_ONLY);

    const PlateSettings empty = fromTrainMap(QVariantMap(), fallback);
    QCOMPARE(empty.plateType, fallback.plateType);
    COMPARE_DOUBLE(empty.radiusMm, fallback.radiusMm, 1e-12);
    QCOMPARE(empty.threadIndex, fallback.threadIndex);
    COMPARE_DOUBLE(empty.screwPitchMm, fallback.screwPitchMm, 1e-12);
    COMPARE_DOUBLE(empty.plateAngleDeg, fallback.plateAngleDeg, 1e-12);
    QCOMPARE(empty.mode, fallback.mode);

    QVariantMap partial;
    partial[QStringLiteral("radius")] = 70.5;
    const PlateSettings merged = fromTrainMap(partial, fallback);
    COMPARE_DOUBLE(merged.radiusMm, 70.5, 1e-12);
    QCOMPARE(merged.plateType, fallback.plateType);
    COMPARE_DOUBLE(merged.screwPitchMm, fallback.screwPitchMm, 1e-12);
}

void TestTiltCorrection::applySettingValidation()
{
    PlateSettings s = threePoint();
    const QString plateType = QStringLiteral("tiltPlateType"), mode = QStringLiteral("tiltMode"),
                                                               radius = QStringLiteral("tiltRadius"), pitch = QStringLiteral("tiltScrewPitch"),
                                                                                                      thread = QStringLiteral("tiltThreadIndex"), angle = QStringLiteral("tiltRotation");

    QCOMPARE(applySetting(s, plateType, 1, false), SETTING_APPLIED);
    QCOMPARE(s.plateType, PLATE_4POINT);
    QCOMPARE(applySetting(s, plateType, QStringLiteral("0"), false), SETTING_APPLIED);   // numbers as strings are fine
    QCOMPARE(s.plateType, PLATE_3POINT);
    QCOMPARE(applySetting(s, plateType, 2, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, plateType, true, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, plateType, QStringLiteral("x"), false), SETTING_INVALID);
    QCOMPARE(s.plateType, PLATE_3POINT);

    QCOMPARE(applySetting(s, mode, 1.0, false), SETTING_APPLIED);   // JSON numbers arrive as doubles
    QCOMPARE(s.mode, MODE_PUSH_ONLY);
    QCOMPARE(applySetting(s, mode, -1, false), SETTING_INVALID);

    QCOMPARE(applySetting(s, radius, MIN_RADIUS_MM, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, radius, MAX_RADIUS_MM, false), SETTING_APPLIED);
    COMPARE_DOUBLE(s.radiusMm, MAX_RADIUS_MM, 1e-12);
    QCOMPARE(applySetting(s, radius, 4.9, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, radius, 100.1, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, radius, 0, false), SETTING_INVALID);
    COMPARE_DOUBLE(s.radiusMm, MAX_RADIUS_MM, 1e-12);

    QCOMPARE(applySetting(s, pitch, MIN_PITCH_MM, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, pitch, MAX_PITCH_MM, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, pitch, 0.05, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, pitch, 2.1, false), SETTING_INVALID);

    QCOMPARE(applySetting(s, thread, 0, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, thread, THREAD_PRESET_COUNT - 1, false), SETTING_APPLIED);
    QCOMPARE(s.threadIndex, THREAD_PRESET_COUNT - 1);
    QCOMPARE(applySetting(s, thread, -1, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, thread, THREAD_PRESET_COUNT, false), SETTING_INVALID);

    QCOMPARE(applySetting(s, angle, -180.0, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, angle, 180.0, false), SETTING_APPLIED);
    QCOMPARE(applySetting(s, angle, 15.5, false), SETTING_APPLIED);
    COMPARE_DOUBLE(s.plateAngleDeg, 15.5, 1e-12);
    QCOMPARE(applySetting(s, angle, 181.0, false), SETTING_INVALID);
    QCOMPARE(applySetting(s, angle, QStringLiteral("abc"), false), SETTING_INVALID);

    QCOMPARE(applySetting(s, QStringLiteral("abInsOptCentres"), true, false), SETTING_UNKNOWN_KEY);
    QCOMPARE(applySetting(s, QStringLiteral("bogus"), 1, false), SETTING_UNKNOWN_KEY);
}

void TestTiltCorrection::applySettingLockedByEta()
{
    // A connected Wanderer ETA fixes radius, pitch and thread preset of a 3-point plate
    PlateSettings s = threePoint();
    QCOMPARE(applySetting(s, QStringLiteral("tiltRadius"), 40.0, true), SETTING_INVALID);
    QCOMPARE(applySetting(s, QStringLiteral("tiltScrewPitch"), 0.7, true), SETTING_INVALID);
    QCOMPARE(applySetting(s, QStringLiteral("tiltThreadIndex"), 3, true), SETTING_INVALID);
    COMPARE_DOUBLE(s.radiusMm, 64.0, 1e-12);
    COMPARE_DOUBLE(s.screwPitchMm, 0.5, 1e-12);
    QCOMPARE(s.threadIndex, 8);
    // Plate type, mode and angle stay adjustable
    QCOMPARE(applySetting(s, QStringLiteral("tiltRotation"), 30.0, true), SETTING_APPLIED);
    QCOMPARE(applySetting(s, QStringLiteral("tiltMode"), MODE_PUSH_ONLY, true), SETTING_APPLIED);
    QCOMPARE(applySetting(s, QStringLiteral("tiltPlateType"), PLATE_4POINT, true), SETTING_APPLIED);
    // A 4-point plate is not an ETA, so its geometry is free again
    QCOMPARE(applySetting(s, QStringLiteral("tiltRadius"), 40.0, true), SETTING_APPLIED);
    COMPARE_DOUBLE(s.radiusMm, 40.0, 1e-12);
}

void TestTiltCorrection::tiltToJson()
{
    const PlateSettings s = threePoint();
    const Result r = compute(lrTilt(), s);
    const QJsonObject json = toJson(r, s, true, false);

    QCOMPARE(json.value(QStringLiteral("valid")).toBool(false), true);
    QCOMPARE(json.value(QStringLiteral("present")).toBool(false), true);
    QCOMPARE(json.value(QStringLiteral("etaConnected")).toBool(true), false);
    QCOMPARE(json.value(QStringLiteral("plateType")).toInt(-1), PLATE_3POINT);
    QCOMPARE(json.value(QStringLiteral("mode")).toInt(-1), MODE_RELATIVE);
    COMPARE_DOUBLE(json.value(QStringLiteral("radiusMm")).toDouble(), 64.0, 1e-12);
    COMPARE_DOUBLE(json.value(QStringLiteral("screwPitchMm")).toDouble(), 0.5, 1e-12);
    QCOMPARE(json.value(QStringLiteral("threadIndex")).toInt(-1), 8);
    COMPARE_DOUBLE(json.value(QStringLiteral("plateAngleDeg")).toDouble(), 0.0, 1e-12);
    COMPARE_DOUBLE(json.value(QStringLiteral("totalTiltMicrons")).toDouble(), 20.0, 1e-9);
    COMPARE_DOUBLE(json.value(QStringLiteral("maxAdjustmentMm")).toDouble(), 0.076980, 1e-6);

    const QJsonArray points = json.value(QStringLiteral("points")).toArray();
    QCOMPARE(points.count(), 3);
    const double angles[3] = { 150.0, 270.0, 30.0 };
    const char *const positions[3] = { "upperRight", "bottom", "upperLeft" };
    for (int i = 0; i < 3; i++)
    {
        const QJsonObject point = points[i].toObject();
        QCOMPARE(point.value(QStringLiteral("index")).toInt(-1), i + 1);
        COMPARE_DOUBLE(point.value(QStringLiteral("angleDeg")).toDouble(), angles[i], 1e-9);
        QCOMPARE(point.value(QStringLiteral("position")).toString(), QString::fromLatin1(positions[i]));
        COMPARE_DOUBLE(point.value(QStringLiteral("adjustmentMm")).toDouble(), r.adjustmentMm[i], 1e-12);
        COMPARE_DOUBLE(point.value(QStringLiteral("turns")).toDouble(), r.turns[i], 1e-12);
    }
    for (auto it = json.constBegin(); it != json.constEnd(); ++it)
        QVERIFY(!it.key().contains(QLatin1Char('_')));
}

QTEST_GUILESS_MAIN(TestTiltCorrection)
