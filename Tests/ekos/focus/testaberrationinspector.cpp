/*
    SPDX-FileCopyrightText: 2026 Alexandre Carmone <acarmone-prestataire@ina.fr>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "ekos/focus/aberrationinspectoranalysis.h"

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

using Ekos::AbInsData;
using Ekos::AberrationInspectorAnalysis;
using Ekos::CurveFitting;
using Ekos::NUM_TILES;
using Ekos::TILE_BL;
using Ekos::TILE_BM;
using Ekos::TILE_BR;
using Ekos::TILE_CL;
using Ekos::TILE_CM;
using Ekos::TILE_CR;
using Ekos::TILE_TL;
using Ekos::TILE_TM;
using Ekos::TILE_TR;

// Headless tests of the Aberration Inspector maths (AberrationInspectorAnalysis). No KStars window, no INDI.
//
// Fixture geometry: 4000 x 3000 px sensor, 4.0 um pixels, 400 px tiles, so the tile centres are at
// (+/-7200, +/-5200) um from the sensor centre, the corner tiles are 8881.44 um from it, the top/bottom tiles
// 5200 um, the left/right tiles 7200 um, and the tilt spans are 14400 um (L-R) and 10400 um (T-B).
class TestAberrationInspector : public QObject
{
        Q_OBJECT

    public:
        TestAberrationInspector() = default;
        ~TestAberrationInspector() override = default;

    private Q_SLOTS:
        void fitHyperbolaExact();
        void fitRoundsSolution();
        void fitCallbackOrderAndState();
        void fitTooFewPoints();
        void fitDatapointMismatch();
        void fitShortVectors();
        void fitWeightsMismatch();
        void fitTwice();
        void numStarsMissing();
        void tableRows();
        void deltaMicronsNotTruncated();
        void backfocusPure();
        void backfocusAllWeighted();
        void adviceThresholds();
        void tiltPureLR();
        void tiltTBPlusBackfocus();
        void excludeTile();
        void excludeCentreFails();
        void unfitTile();
        void optCentresWeights();
        void optCentresOffsetsMissing();
        void setSolutionSeam();
        void invalidSelection();
        void optionAndKeyStrings();
        void tileSelectionFromVariant();
        void toJsonShape();
};

#include "testaberrationinspector.moc"

namespace
{

#define COMPARE_DOUBLE(actual, expected, tolerance) \
    QVERIFY2(std::abs((actual) - (expected)) <= (tolerance), \
             qPrintable(QString("%1: actual %2, expected %3").arg(#actual).arg(actual, 0, 'f', 6).arg(expected, 0, 'f', 6)))

using Selection = AberrationInspectorAnalysis::TileSelection;
using Settings = AberrationInspectorAnalysis::Settings;
using Result = AberrationInspectorAnalysis::Result;

// Per-tile solutions (focuser steps) of the three scenarios used below, in tile order TL, T, TR, L, C, R, BL, B, BR.
// A: pure backfocus, every outer tile focuses 10 steps further out than the centre.
const int SCENARIO_A[NUM_TILES] = { 1010, 1010, 1010, 1010, 1000, 1010, 1010, 1010, 1010 };
// B: pure left-to-right tilt, left column +5, right column -5.
const int SCENARIO_B[NUM_TILES] = { 1005, 1000, 995, 1005, 1000, 995, 1005, 1000, 995 };
// C: top-to-bottom tilt plus backfocus: top row +12, middle row +4, bottom row -4.
const int SCENARIO_C[NUM_TILES] = { 1012, 1012, 1012, 1004, 1000, 1004, 996, 996, 996 };

AbInsData makeData(double focuserStepMicrons)
{
    AbInsData data;
    data.run = 1;
    data.curveFit = CurveFitting::FOCUS_HYPERBOLA;
    data.useWeights = false;
    data.optDir = CurveFitting::OPTIMISATION_MINIMISE;
    data.sensorWidth = 4000;
    data.sensorHeight = 3000;
    data.pixelSize = 4.0;
    data.tileWidth = 400;
    data.focuserStepMicrons = focuserStepMicrons;
    data.yAxisLabel = QStringLiteral("HFR");
    data.starUnits = 1.0;
    data.cfzSteps = 10.0;
    data.isPositionBased = true;
    data.opticalTrainID = 0;
    return data;
}

struct Fixture
{
    QVector<int> positions;
    QVector<QVector<double>> measures;
    QVector<QVector<double>> weights;
    QVector<QVector<int>> numStars;
    QVector<QPoint> offsets;
};

// Synthetic autofocus run: positions 900..1100 in steps of 20 and, per tile, an exact hyperbola
// y = sqrt(1 + ((x - solution) / 50)^2) + 1 whose minimum of 2.0 is at the tile's solution.
Fixture makeFixture(const double solutions[NUM_TILES], int numPoints = 11)
{
    Fixture f;
    for (int i = 0; i < numPoints; i++)
        f.positions.append(900 + 20 * i);
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QVector<double> measures, weights;
        for (int pos : f.positions)
        {
            const double x = (pos - solutions[tile]) / 50.0;
            measures.append(std::sqrt(1.0 + x * x) + 1.0);
            weights.append(1.0);
        }
        f.measures.append(measures);
        f.weights.append(weights);
        f.numStars.append({ 10 + tile, 12 + tile, 11 + tile });
        f.offsets.append(QPoint(0, 0));
    }
    return f;
}

Fixture makeFixture(const int solutions[NUM_TILES], int numPoints = 11)
{
    double asDouble[NUM_TILES];
    for (int tile = 0; tile < NUM_TILES; tile++)
        asDouble[tile] = solutions[tile];
    return makeFixture(asDouble, numPoints);
}

std::unique_ptr<AberrationInspectorAnalysis> makeAnalysis(const Fixture &f, double step = 2.0)
{
    return std::unique_ptr<AberrationInspectorAnalysis>(new AberrationInspectorAnalysis(makeData(step), f.positions, f.measures,
           f.weights, f.numStars, f.offsets));
}

// Inject known solutions instead of running the solver, so the maths can be checked against closed-form values.
void inject(AberrationInspectorAnalysis &analysis, const int solutions[NUM_TILES])
{
    for (int tile = 0; tile < NUM_TILES; tile++)
        analysis.setTileFit(tile, { true, solutions[tile], 2.0, 1.0 });
}

Settings settingsFor(Selection selection, bool optCentres = false)
{
    Settings settings;
    settings.tileSelection = selection;
    settings.optCentres = optCentres;
    return settings;
}

}

void TestAberrationInspector::fitHyperbolaExact()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f);
    analysis->fit();

    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        const auto &fit = analysis->tileFit(tile);
        QVERIFY2(fit.fit, qPrintable(QString("tile %1 did not fit").arg(tile)));
        QVERIFY2(std::abs(fit.solution - SCENARIO_B[tile]) <= 1,
                 qPrintable(QString("tile %1 solution %2 expected %3").arg(tile).arg(fit.solution).arg(SCENARIO_B[tile])));
        COMPARE_DOUBLE(fit.minMeasure, 2.0, 1e-3);
        QVERIFY2(fit.R2 > 0.999, qPrintable(QString("tile %1 R2 %2").arg(tile).arg(fit.R2)));
    }
    QCOMPARE(analysis->datapointCount(), 11);
    QVERIFY(analysis->analyse(settingsFor(Selection::TILES_ALL)).warnings.isEmpty());
}

void TestAberrationInspector::fitCallbackOrderAndState()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f);

    QVector<int> order;
    QVector<double> valueAtSolution;
    analysis->fit([&](int tile, CurveFitting * curve, const AberrationInspectorAnalysis::TileFit & fit)
    {
        order.append(tile);
        QVERIFY(curve != nullptr);
        QVERIFY(fit.fit);
        // The solver still holds this tile's solution, so the curve can be sampled, as the plot does
        valueAtSolution.append(curve->f(SCENARIO_B[tile]));
    });

    QCOMPARE(order.count(), static_cast<int>(NUM_TILES));
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QCOMPARE(order[tile], tile);
        COMPARE_DOUBLE(valueAtSolution[tile], 2.0, 1e-3);
    }
}

void TestAberrationInspector::fitTooFewPoints()
{
    const Fixture f = makeFixture(SCENARIO_B, 3);
    auto analysis = makeAnalysis(f);
    int callbacks = 0;
    analysis->fit([&](int, CurveFitting *, const AberrationInspectorAnalysis::TileFit &)
    {
        callbacks++;
    });
    QCOMPARE(callbacks, static_cast<int>(NUM_TILES));

    for (int tile = 0; tile < NUM_TILES; tile++)
        QVERIFY(!analysis->tileFit(tile).fit);

    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QVERIFY(result.warnings.contains(QStringLiteral("insufficientDatapoints")));
    QVERIFY(!result.backfocusOK);
    QVERIFY(!result.tiltOK);
    QVERIFY(!result.resultsOK);
    QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NONE);
}

void TestAberrationInspector::fitDatapointMismatch()
{
    Fixture f = makeFixture(SCENARIO_B);
    f.measures[1].removeLast();
    auto analysis = makeAnalysis(f);
    analysis->fit();

    for (int tile = 0; tile < NUM_TILES; tile++)
        QCOMPARE(analysis->tileFit(tile).fit, tile != 1);

    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QVERIFY(result.warnings.contains(QStringLiteral("datapointMismatch")));
    QVERIFY(!result.rows[1].fit);
    // The other tiles still give a full result
    QVERIFY(result.backfocusOK);
    QVERIFY(result.tiltOK);
}

void TestAberrationInspector::tableRows()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    const Result all = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QCOMPARE(all.rows[0].tile, 0);
    QCOMPARE(all.rows[0].solution, 1005);
    QCOMPARE(all.rows[0].deltaTicks, 5);
    COMPARE_DOUBLE(all.rows[0].deltaMicrons, 10.0, 1e-9);
    QCOMPARE(all.rows[2].deltaTicks, -5);
    COMPARE_DOUBLE(all.rows[2].deltaMicrons, -10.0, 1e-9);
    QCOMPARE(all.rows[4].deltaTicks, 0);
    COMPARE_DOUBLE(all.rows[4].deltaMicrons, 0.0, 1e-9);
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QCOMPARE(all.rows[tile].minStars, 10 + tile);
        QCOMPARE(all.rows[tile].maxStars, 12 + tile);
        QVERIFY(all.rows[tile].fit);
        QVERIFY(!all.rows[tile].excluded);
        QVERIFY(all.rows[tile].used);
        QVERIFY(all.useTile[tile]);
        COMPARE_DOUBLE(all.rows[tile].R2, 1.0, 1e-9);
        COMPARE_DOUBLE(all.rows[tile].minMeasure, 2.0, 1e-9);
    }

    const Result outer = analysis->analyse(settingsFor(Selection::TILES_OUTER_CORNERS));
    const Result diamond = analysis->analyse(settingsFor(Selection::TILES_INNER_DIAMOND));
    const bool outerExpected[NUM_TILES] = { true, false, true, false, true, false, true, false, true };
    const bool diamondExpected[NUM_TILES] = { false, true, false, true, true, true, false, true, false };
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QCOMPARE(outer.useTile[tile], outerExpected[tile]);
        QCOMPARE(outer.rows[tile].used, outerExpected[tile]);
        QCOMPARE(diamond.useTile[tile], diamondExpected[tile]);
        QCOMPARE(diamond.rows[tile].used, diamondExpected[tile]);
    }

    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_ALL, 7), 7);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_OUTER_CORNERS, 3), 6);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_INNER_DIAMOND, 0), 1);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_INNER_DIAMOND, 1), 3);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_INNER_DIAMOND, 2), 4);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_INNER_DIAMOND, 3), 5);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_INNER_DIAMOND, 4), 7);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_ALL, 9), -1);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(Selection::TILES_ALL, -1), -1);
}

void TestAberrationInspector::deltaMicronsNotTruncated()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 1.5);
    inject(*analysis, SCENARIO_B);
    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    COMPARE_DOUBLE(result.rows[0].deltaMicrons, 7.5, 1e-9);
    COMPARE_DOUBLE(result.rows[2].deltaMicrons, -7.5, 1e-9);
}

void TestAberrationInspector::backfocusPure()
{
    const Fixture f = makeFixture(SCENARIO_A);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_A);

    for (auto selection :
            {
                Selection::TILES_ALL, Selection::TILES_OUTER_CORNERS, Selection::TILES_INNER_DIAMOND
            })
    {
        const Result result = analysis->analyse(settingsFor(selection));
        QVERIFY(result.backfocusOK);
        COMPARE_DOUBLE(result.backfocusMicrons, -20.0, 1e-6);
        QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NEARER);
        QVERIFY(result.tiltOK);
        COMPARE_DOUBLE(result.lrMicrons, 0.0, 1e-9);
        COMPARE_DOUBLE(result.tbMicrons, 0.0, 1e-9);
        COMPARE_DOUBLE(result.diagonalMicrons, 0.0, 1e-9);
        COMPARE_DOUBLE(result.lrTiltPercent, 0.0, 1e-9);
        COMPARE_DOUBLE(result.tbTiltPercent, 0.0, 1e-9);
        QVERIFY(result.resultsOK);
        for (int tile = 0; tile < NUM_TILES; tile++)
            COMPARE_DOUBLE(result.deltasMicrons[tile], tile == TILE_CM ? 0.0 : -20.0, 1e-9);
    }
}

void TestAberrationInspector::adviceThresholds()
{
    const Fixture f = makeFixture(SCENARIO_A);
    auto analysis = makeAnalysis(f, 0.4);

    auto withOuter = [&](int outer)
    {
        int solutions[NUM_TILES];
        for (int tile = 0; tile < NUM_TILES; tile++)
            solutions[tile] = tile == TILE_CM ? 1000 : outer;
        inject(*analysis, solutions);
        return analysis->analyse(settingsFor(Selection::TILES_ALL));
    };

    Result result = withOuter(998);
    COMPARE_DOUBLE(result.backfocusMicrons, 0.8, 1e-9);
    QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NONE);

    result = withOuter(997);
    COMPARE_DOUBLE(result.backfocusMicrons, 1.2, 1e-9);
    QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_AWAY);

    result = withOuter(1003);
    COMPARE_DOUBLE(result.backfocusMicrons, -1.2, 1e-9);
    QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NEARER);
}

void TestAberrationInspector::tiltPureLR()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    for (auto selection :
            {
                Selection::TILES_ALL, Selection::TILES_OUTER_CORNERS, Selection::TILES_INNER_DIAMOND
            })
    {
        const Result result = analysis->analyse(settingsFor(selection));
        QVERIFY(result.backfocusOK);
        COMPARE_DOUBLE(result.backfocusMicrons, 0.0, 1e-6);
        QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NONE);
        QVERIFY(result.tiltOK);
        COMPARE_DOUBLE(result.lrMicrons, -20.0, 1e-6);
        COMPARE_DOUBLE(result.tbMicrons, 0.0, 1e-6);
        COMPARE_DOUBLE(result.diagonalMicrons, 20.0, 1e-6);
        COMPARE_DOUBLE(result.lrSpanMicrons, 14400.0, 1e-9);
        COMPARE_DOUBLE(result.tbSpanMicrons, 10400.0, 1e-9);
        COMPARE_DOUBLE(result.lrTiltPercent, -20.0 / 14400.0 * 100.0, 1e-9);
        COMPARE_DOUBLE(result.tbTiltPercent, 0.0, 1e-9);
        COMPARE_DOUBLE(result.diagonalTiltPercent, 20.0 / 14400.0 * 100.0, 1e-9);
        QVERIFY(result.resultsOK);
    }
}

void TestAberrationInspector::tiltTBPlusBackfocus()
{
    const Fixture f = makeFixture(SCENARIO_C);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_C);

    for (auto selection :
            {
                Selection::TILES_ALL, Selection::TILES_OUTER_CORNERS, Selection::TILES_INNER_DIAMOND
            })
    {
        const Result result = analysis->analyse(settingsFor(selection));
        QVERIFY(result.backfocusOK);
        // Every opposite pair of tiles averages 1004 steps, so the weighted mean is 1004 for all selections
        COMPARE_DOUBLE(result.backfocusMicrons, -8.0, 1e-6);
        QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NEARER);
        QVERIFY(result.tiltOK);
        COMPARE_DOUBLE(result.lrMicrons, 0.0, 1e-6);
        COMPARE_DOUBLE(result.tbMicrons, -32.0, 1e-6);
        COMPARE_DOUBLE(result.diagonalMicrons, 32.0, 1e-6);
        COMPARE_DOUBLE(result.tbTiltPercent, -32.0 / 10400.0 * 100.0, 1e-9);
        QVERIFY(result.resultsOK);
    }
}

void TestAberrationInspector::excludeTile()
{
    const Fixture f = makeFixture(SCENARIO_C);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_C);

    Settings settings = settingsFor(Selection::TILES_OUTER_CORNERS);
    settings.exclude[TILE_TL] = true;
    const Result result = analysis->analyse(settings);
    QVERIFY(result.rows[TILE_TL].excluded);
    QVERIFY(result.backfocusOK);
    // Remaining corners TR, BL, BR: (1012 + 996 + 996) / 3 = 1001.333 -> (1000 - 1001.333) * 2
    COMPARE_DOUBLE(result.backfocusMicrons, (1000.0 - (1012.0 + 996.0 + 996.0) / 3.0) * 2.0, 1e-6);
    QVERIFY(result.tiltOK);
    // Left column is now BL only (+8 um), right column TR (-24 um) and BR (+8 um)
    COMPARE_DOUBLE(result.lrMicrons, 16.0, 1e-6);
    COMPARE_DOUBLE(result.tbMicrons, -32.0, 1e-6);
    COMPARE_DOUBLE(result.lrTiltPercent, 16.0 / 14400.0 * 100.0, 1e-9);
    COMPARE_DOUBLE(result.diagonalMicrons, std::hypot(16.0, 32.0), 1e-6);

    // Excluding a tile that is not part of the selection changes nothing
    Settings diamond = settingsFor(Selection::TILES_INNER_DIAMOND);
    diamond.exclude[TILE_TL] = true;
    const Result unchanged = analysis->analyse(diamond);
    COMPARE_DOUBLE(unchanged.backfocusMicrons, -8.0, 1e-6);
    COMPARE_DOUBLE(unchanged.tbMicrons, -32.0, 1e-6);
    COMPARE_DOUBLE(unchanged.lrMicrons, 0.0, 1e-6);
}

void TestAberrationInspector::excludeCentreFails()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    Settings settings = settingsFor(Selection::TILES_ALL);
    settings.exclude[TILE_CM] = true;
    const Result result = analysis->analyse(settings);
    QVERIFY(!result.backfocusOK);
    QVERIFY(!result.tiltOK);
    QVERIFY(!result.resultsOK);
    COMPARE_DOUBLE(result.backfocusMicrons, 0.0, 1e-9);
    QCOMPARE(result.advice, AberrationInspectorAnalysis::ADVICE_NONE);
}

void TestAberrationInspector::unfitTile()
{
    const Fixture f = makeFixture(SCENARIO_C);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_C);
    analysis->setTileFit(TILE_CL, { false, 0, 0.0, 0.0 });

    // Inner diamond: backfocus still works from the three remaining tiles, but the left column has no usable tile
    const Result diamond = analysis->analyse(settingsFor(Selection::TILES_INNER_DIAMOND));
    QVERIFY(!diamond.rows[TILE_CL].fit);
    QVERIFY(diamond.backfocusOK);
    COMPARE_DOUBLE(diamond.backfocusMicrons, -8.0, 1e-6);
    QVERIFY(!diamond.tiltOK);
    QVERIFY(!diamond.resultsOK);

    // All tiles: the corners still give a left column average, and every remaining pair still averages 1004
    const Result all = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QVERIFY(all.backfocusOK);
    COMPARE_DOUBLE(all.backfocusMicrons, -8.0, 1e-6);
    QVERIFY(all.tiltOK);
    COMPARE_DOUBLE(all.lrMicrons, 0.0, 1e-6);
    COMPARE_DOUBLE(all.tbMicrons, -32.0, 1e-6);
    QVERIFY(all.resultsOK);
}

void TestAberrationInspector::optCentresWeights()
{
    Fixture f = makeFixture(SCENARIO_B);
    // Stars in the left tile cluster 300 px towards the centre
    f.offsets[TILE_CL] = QPoint(300, 0);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    const QVector2D tl = analysis->tileCentreMicrons(TILE_TL, false);
    COMPARE_DOUBLE(tl.x(), -7200.0, 1e-6);
    COMPARE_DOUBLE(tl.y(), 5200.0, 1e-6);
    const QVector2D br = analysis->tileCentreMicrons(TILE_BR, false);
    COMPARE_DOUBLE(br.x(), 7200.0, 1e-6);
    COMPARE_DOUBLE(br.y(), -5200.0, 1e-6);
    const QVector2D cm = analysis->tileCentreMicrons(TILE_CM, true);
    COMPARE_DOUBLE(cm.x(), 0.0, 1e-6);
    COMPARE_DOUBLE(cm.y(), 0.0, 1e-6);
    const QVector2D clOff = analysis->tileCentreMicrons(TILE_CL, false);
    COMPARE_DOUBLE(clOff.x(), -7200.0, 1e-6);
    const QVector2D clOn = analysis->tileCentreMicrons(TILE_CL, true);
    COMPARE_DOUBLE(clOn.x(), -6000.0, 1e-6);
    COMPARE_DOUBLE(clOn.y(), 0.0, 1e-6);

    const Result off = analysis->analyse(settingsFor(Selection::TILES_INNER_DIAMOND, false));
    QVERIFY(!off.optCentres);
    COMPARE_DOUBLE(off.backfocusMicrons, 0.0, 1e-6);

    // Weighted mean with the left tile at 6000 um: (1000*5200 + 1005*6000 + 995*7200 + 1000*5200) / 23600
    const Result on = analysis->analyse(settingsFor(Selection::TILES_INNER_DIAMOND, true));
    QVERIFY(on.optCentres);
    QVERIFY(on.warnings.isEmpty());
    const double weightedMean = (1000.0 * 5200.0 + 1005.0 * 6000.0 + 995.0 * 7200.0 + 1000.0 * 5200.0) / 23600.0;
    COMPARE_DOUBLE(on.backfocusMicrons, (1000.0 - weightedMean) * 2.0, 1e-6);
    // Tilt does not depend on the tile centres
    COMPARE_DOUBLE(on.lrMicrons, -20.0, 1e-6);
    COMPARE_DOUBLE(on.tbMicrons, 0.0, 1e-6);
}

void TestAberrationInspector::optCentresOffsetsMissing()
{
    Fixture f = makeFixture(SCENARIO_B);
    f.offsets.clear();
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    const Result on = analysis->analyse(settingsFor(Selection::TILES_ALL, true));
    QVERIFY(!on.optCentres);
    QVERIFY(on.warnings.contains(QStringLiteral("tileOffsetsUnavailable")));
    const Result off = analysis->analyse(settingsFor(Selection::TILES_ALL, false));
    QVERIFY(off.warnings.isEmpty());
    COMPARE_DOUBLE(on.backfocusMicrons, off.backfocusMicrons, 1e-9);
    COMPARE_DOUBLE(on.lrMicrons, off.lrMicrons, 1e-9);
    // Asking for the offset of a tile without offsets falls back to the geometric centre
    COMPARE_DOUBLE(analysis->tileCentreMicrons(TILE_TR, true).x(), 7200.0, 1e-6);

    // A partial set of offsets (fewer than one per tile) is not used either, for any tile
    Fixture g = makeFixture(SCENARIO_B);
    g.offsets.resize(5);
    g.offsets[TILE_TL] = QPoint(300, 0);
    auto partial = makeAnalysis(g, 2.0);
    inject(*partial, SCENARIO_B);
    COMPARE_DOUBLE(partial->tileCentreMicrons(TILE_TL, true).x(), -7200.0, 1e-6);
    const Result partialOn = partial->analyse(settingsFor(Selection::TILES_ALL, true));
    QVERIFY(!partialOn.optCentres);
    QVERIFY(partialOn.warnings.contains(QStringLiteral("tileOffsetsUnavailable")));
    COMPARE_DOUBLE(partialOn.backfocusMicrons, off.backfocusMicrons, 1e-9);
}

void TestAberrationInspector::setSolutionSeam()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);
    analysis->setSolution(TILE_TL, 1007);
    QCOMPARE(analysis->tileFit(TILE_TL).solution, 1007);
    QVERIFY(analysis->tileFit(TILE_TL).fit);
    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QCOMPARE(result.rows[TILE_TL].deltaTicks, 7);
    // Out of range tiles are ignored
    analysis->setSolution(NUM_TILES, 1);
    analysis->setSolution(-1, 1);
    QVERIFY(!analysis->tileFit(NUM_TILES).fit);
}

void TestAberrationInspector::optionAndKeyStrings()
{
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(i18n("All")), Selection::TILES_ALL);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(i18n("Centre and outer corners")),
             Selection::TILES_OUTER_CORNERS);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(i18n("Centre and inner diamond")),
             Selection::TILES_INNER_DIAMOND);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(QStringLiteral("All")), Selection::TILES_ALL);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(QStringLiteral("Centre and inner diamond")),
             Selection::TILES_INNER_DIAMOND);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(QString()), Selection::TILES_OUTER_CORNERS);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(QStringLiteral("garbage")), Selection::TILES_OUTER_CORNERS);

    for (auto selection :
            {
                Selection::TILES_ALL, Selection::TILES_OUTER_CORNERS, Selection::TILES_INNER_DIAMOND
            })
    {
        QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromOption(AberrationInspectorAnalysis::tileSelectionToOption(
                     selection)), selection);
        bool ok = false;
        QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromKey(AberrationInspectorAnalysis::tileSelectionToKey(selection),
                 &ok), selection);
        QVERIFY(ok);
    }
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionToKey(Selection::TILES_ALL), QStringLiteral("all"));
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionToKey(Selection::TILES_OUTER_CORNERS), QStringLiteral("outerCorners"));
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionToKey(Selection::TILES_INNER_DIAMOND), QStringLiteral("innerDiamond"));

    bool ok = true;
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromKey(QStringLiteral("bogus"), &ok), Selection::TILES_OUTER_CORNERS);
    QVERIFY(!ok);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionFromKey(QStringLiteral("innerDiamond")), Selection::TILES_INNER_DIAMOND);

    QCOMPARE(AberrationInspectorAnalysis::adviceKey(AberrationInspectorAnalysis::ADVICE_NONE), QStringLiteral("none"));
    QCOMPARE(AberrationInspectorAnalysis::adviceKey(AberrationInspectorAnalysis::ADVICE_NEARER), QStringLiteral("nearer"));
    QCOMPARE(AberrationInspectorAnalysis::adviceKey(AberrationInspectorAnalysis::ADVICE_AWAY), QStringLiteral("away"));

    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_HYPERBOLA), QStringLiteral("hyperbola"));
    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_PARABOLA), QStringLiteral("parabola"));
    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_QUADRATIC), QStringLiteral("quadratic"));
    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_2DGAUSSIAN), QStringLiteral("gaussian2D"));
    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_3DGAUSSIAN), QStringLiteral("gaussian3D"));
    QCOMPARE(AberrationInspectorAnalysis::curveFitKey(CurveFitting::FOCUS_PLANE), QStringLiteral("plane"));
}

void TestAberrationInspector::tileSelectionFromVariant()
{
    Selection selection = Selection::TILES_ALL;
    // Combo box index, as a number, a JSON double or a numeric string
    QVERIFY(AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(2), &selection));
    QCOMPARE(selection, Selection::TILES_INNER_DIAMOND);
    QVERIFY(AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(1.0), &selection));
    QCOMPARE(selection, Selection::TILES_OUTER_CORNERS);
    QVERIFY(AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(QStringLiteral("0")), &selection));
    QCOMPARE(selection, Selection::TILES_ALL);
    // Machine-readable key
    QVERIFY(AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(QStringLiteral("innerDiamond")), &selection));
    QCOMPARE(selection, Selection::TILES_INNER_DIAMOND);
    // Invalid values leave the selection untouched
    QVERIFY(!AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(3), &selection));
    QVERIFY(!AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(-1), &selection));
    QVERIFY(!AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(QStringLiteral("bogus")), &selection));
    QVERIFY(!AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(true), &selection));
    QVERIFY(!AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(), &selection));
    QCOMPARE(selection, Selection::TILES_INNER_DIAMOND);
    QVERIFY(AberrationInspectorAnalysis::tileSelectionFromVariant(QVariant(QStringLiteral("all")), nullptr));
}

namespace
{
bool keysAreCamelCase(const QJsonObject &object)
{
    for (auto it = object.constBegin(); it != object.constEnd(); ++it)
    {
        if (it.key().contains(QLatin1Char('_')))
            return false;
        if (it.value().isObject() && !keysAreCamelCase(it.value().toObject()))
            return false;
        if (it.value().isArray())
        {
            for (const auto &item : it.value().toArray())
                if (item.isObject() && !keysAreCamelCase(item.toObject()))
                    return false;
        }
    }
    return true;
}
}

void TestAberrationInspector::toJsonShape()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);
    const Result result = analysis->analyse(settingsFor(Selection::TILES_OUTER_CORNERS));
    const QJsonObject json = analysis->toJson(result);

    QVERIFY(keysAreCamelCase(json));
    QCOMPARE(json.value(QStringLiteral("run")).toInt(), 1);
    QCOMPARE(json.value(QStringLiteral("curveFit")).toString(), QStringLiteral("hyperbola"));
    QCOMPARE(json.value(QStringLiteral("useWeights")).toBool(true), false);
    QCOMPARE(json.value(QStringLiteral("datapoints")).toInt(), 11);
    QCOMPARE(json.value(QStringLiteral("yAxisLabel")).toString(), QStringLiteral("HFR"));
    COMPARE_DOUBLE(json.value(QStringLiteral("focuserStepMicrons")).toDouble(), 2.0, 1e-9);
    COMPARE_DOUBLE(json.value(QStringLiteral("cfzSteps")).toDouble(), 10.0, 1e-9);
    QCOMPARE(json.value(QStringLiteral("tileSelection")).toString(), QStringLiteral("outerCorners"));
    QCOMPARE(json.value(QStringLiteral("optCentres")).toBool(true), false);
    QVERIFY(json.value(QStringLiteral("warnings")).isArray());
    QCOMPARE(json.value(QStringLiteral("warnings")).toArray().count(), 0);
    QCOMPARE(json.value(QStringLiteral("resultsOK")).toBool(false), true);

    const QJsonObject sensor = json.value(QStringLiteral("sensor")).toObject();
    QCOMPARE(sensor.value(QStringLiteral("widthPx")).toInt(), 4000);
    QCOMPARE(sensor.value(QStringLiteral("heightPx")).toInt(), 3000);
    COMPARE_DOUBLE(sensor.value(QStringLiteral("pixelSizeMicrons")).toDouble(), 4.0, 1e-9);
    QCOMPARE(sensor.value(QStringLiteral("tileWidthPx")).toInt(), 400);
    COMPARE_DOUBLE(sensor.value(QStringLiteral("lrSpanMicrons")).toDouble(), 14400.0, 1e-9);
    COMPARE_DOUBLE(sensor.value(QStringLiteral("tbSpanMicrons")).toDouble(), 10400.0, 1e-9);

    const QJsonArray tiles = json.value(QStringLiteral("tiles")).toArray();
    QCOMPARE(tiles.count(), static_cast<int>(NUM_TILES));
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        const QJsonObject item = tiles[tile].toObject();
        QCOMPARE(item.value(QStringLiteral("id")).toInt(-1), tile);
        QCOMPARE(item.value(QStringLiteral("name")).toString(), Ekos::TILE_NAME[tile]);
        QVERIFY(item.contains(QStringLiteral("used")));
        QVERIFY(item.contains(QStringLiteral("fit")));
        QVERIFY(item.contains(QStringLiteral("excluded")));
        QVERIFY(item.contains(QStringLiteral("solution")));
        QVERIFY(item.contains(QStringLiteral("deltaTicks")));
        QVERIFY(item.contains(QStringLiteral("deltaMicrons")));
        QVERIFY(item.contains(QStringLiteral("minStars")));
        QVERIFY(item.contains(QStringLiteral("maxStars")));
        QVERIFY(item.contains(QStringLiteral("r2")));
        QVERIFY(item.contains(QStringLiteral("minMeasure")));
    }
    QCOMPARE(tiles[0].toObject().value(QStringLiteral("solution")).toInt(), 1005);
    QCOMPARE(tiles[0].toObject().value(QStringLiteral("deltaTicks")).toInt(), 5);
    COMPARE_DOUBLE(tiles[0].toObject().value(QStringLiteral("deltaMicrons")).toDouble(), 10.0, 1e-9);
    QCOMPARE(tiles[1].toObject().value(QStringLiteral("used")).toBool(true), false);
    QCOMPARE(tiles[0].toObject().value(QStringLiteral("used")).toBool(false), true);
    QCOMPARE(tiles[0].toObject().value(QStringLiteral("minStars")).toInt(), 10);
    QCOMPARE(tiles[0].toObject().value(QStringLiteral("maxStars")).toInt(), 12);

    const QJsonObject backfocus = json.value(QStringLiteral("backfocus")).toObject();
    QCOMPARE(backfocus.value(QStringLiteral("ok")).toBool(false), true);
    COMPARE_DOUBLE(backfocus.value(QStringLiteral("deltaMicrons")).toDouble(), 0.0, 1e-6);
    QCOMPARE(backfocus.value(QStringLiteral("advice")).toString(), QStringLiteral("none"));

    const QJsonObject tilt = json.value(QStringLiteral("tilt")).toObject();
    QCOMPARE(tilt.value(QStringLiteral("ok")).toBool(false), true);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("lrMicrons")).toDouble(), -20.0, 1e-6);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("tbMicrons")).toDouble(), 0.0, 1e-6);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("diagonalMicrons")).toDouble(), 20.0, 1e-6);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("lrPercent")).toDouble(), -20.0 / 14400.0 * 100.0, 1e-9);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("tbPercent")).toDouble(), 0.0, 1e-9);
    COMPARE_DOUBLE(tilt.value(QStringLiteral("diagonalPercent")).toDouble(), 20.0 / 14400.0 * 100.0, 1e-9);
}

void TestAberrationInspector::fitRoundsSolution()
{
    // 1004.6 separates rounding (1005) from truncation (1004); 995.4 separates rounding (995) from ceiling (996)
    const double solutions[NUM_TILES] = { 1004.6, 995.4, 1004.6, 995.4, 1000.0, 995.4, 1004.6, 995.4, 1004.6 };
    const int expected[NUM_TILES] = { 1005, 995, 1005, 995, 1000, 995, 1005, 995, 1005 };
    const Fixture f = makeFixture(solutions);
    auto analysis = makeAnalysis(f);
    analysis->fit();
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QVERIFY2(analysis->tileFit(tile).fit, qPrintable(QString("tile %1 did not fit").arg(tile)));
        QCOMPARE(analysis->tileFit(tile).solution, expected[tile]);
    }
}

void TestAberrationInspector::fitShortVectors()
{
    // Measures and weights for the first five tiles only
    Fixture f = makeFixture(SCENARIO_B);
    f.measures.resize(5);
    f.weights.resize(5);
    auto analysis = makeAnalysis(f);

    int callbacks = 0;
    int skipped = 0;
    analysis->fit([&](int tile, CurveFitting * curve, const AberrationInspectorAnalysis::TileFit & fit)
    {
        callbacks++;
        if (curve == nullptr)
            skipped++;
        // Skipped tiles get no solver (and no solution); fitted tiles get the live solver
        QCOMPARE(curve != nullptr, tile < 5);
        QCOMPARE(fit.fit, tile < 5);
    });
    QCOMPARE(callbacks, static_cast<int>(NUM_TILES));
    QCOMPARE(skipped, 4);
    for (int tile = 0; tile < NUM_TILES; tile++)
        QCOMPARE(analysis->tileFit(tile).fit, tile < 5);

    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QCOMPARE(result.warnings.count(QStringLiteral("datapointMismatch")), 1);
    // The centre and the top row still give a backfocus, but the bottom row has no usable tile for the tilt
    QVERIFY(result.backfocusOK);
    QVERIFY(!result.tiltOK);
    QVERIFY(!result.resultsOK);
}

void TestAberrationInspector::fitWeightsMismatch()
{
    Fixture f = makeFixture(SCENARIO_B);
    f.weights[2].removeLast();
    auto analysis = makeAnalysis(f);
    analysis->fit();
    for (int tile = 0; tile < NUM_TILES; tile++)
        QCOMPARE(analysis->tileFit(tile).fit, tile != 2);
    QVERIFY(analysis->analyse(settingsFor(Selection::TILES_ALL)).warnings.contains(QStringLiteral("datapointMismatch")));
}

void TestAberrationInspector::fitTwice()
{
    Fixture f = makeFixture(SCENARIO_B);
    f.measures[1].removeLast();
    auto analysis = makeAnalysis(f);
    analysis->fit();
    AberrationInspectorAnalysis::TileFit first[NUM_TILES];
    for (int tile = 0; tile < NUM_TILES; tile++)
        first[tile] = analysis->tileFit(tile);

    // A second fit resets the previous fits and warnings instead of accumulating them
    analysis->fit();
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QCOMPARE(analysis->tileFit(tile).fit, first[tile].fit);
        QCOMPARE(analysis->tileFit(tile).solution, first[tile].solution);
        COMPARE_DOUBLE(analysis->tileFit(tile).minMeasure, first[tile].minMeasure, 1e-9);
    }
    const Result result = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QCOMPARE(result.warnings.count(QStringLiteral("datapointMismatch")), 1);
    QCOMPARE(result.warnings.count(), 1);
}

void TestAberrationInspector::numStarsMissing()
{
    Fixture f = makeFixture(SCENARIO_B);
    f.numStars.clear();
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);
    const Result none = analysis->analyse(settingsFor(Selection::TILES_ALL));
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QCOMPARE(none.rows[tile].minStars, 0);
        QCOMPARE(none.rows[tile].maxStars, 0);
    }
    QVERIFY(none.backfocusOK);
    QVERIFY(none.tiltOK);

    Fixture g = makeFixture(SCENARIO_B);
    g.numStars[TILE_CL].clear();
    auto partial = makeAnalysis(g, 2.0);
    inject(*partial, SCENARIO_B);
    const Result some = partial->analyse(settingsFor(Selection::TILES_ALL));
    QCOMPARE(some.rows[TILE_CL].minStars, 0);
    QCOMPARE(some.rows[TILE_CL].maxStars, 0);
    QCOMPARE(some.rows[TILE_TL].minStars, 10);
    QCOMPARE(some.rows[TILE_TL].maxStars, 12);
}

void TestAberrationInspector::backfocusAllWeighted()
{
    // Corners differ from the edge tiles, so the distance weighting of "all tiles" matters
    const int solutions[NUM_TILES] = { 1010, 1000, 1010, 1000, 1000, 1000, 1010, 1000, 1010 };
    Fixture f = makeFixture(solutions);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, solutions);

    const double cornerDistance = std::hypot(7200.0, 5200.0);
    const double weightedMean = (4.0 * 1010.0 * cornerDistance + 2.0 * 1000.0 * 5200.0 + 2.0 * 1000.0 * 7200.0)
                                / (4.0 * cornerDistance + 2.0 * 5200.0 + 2.0 * 7200.0);
    const Result all = analysis->analyse(settingsFor(Selection::TILES_ALL));
    QVERIFY(all.backfocusOK);
    COMPARE_DOUBLE(all.backfocusMicrons, (1000.0 - weightedMean) * 2.0, 1e-6);
    // A plain mean of the eight outer tiles would give -10 um
    QVERIFY(std::abs(all.backfocusMicrons + 10.0) > 1.0);
    // The outer corners are all at the same distance: plain mean of the four corners
    COMPARE_DOUBLE(analysis->analyse(settingsFor(Selection::TILES_OUTER_CORNERS)).backfocusMicrons, -20.0, 1e-6);
    // The inner diamond tiles all equal the centre
    COMPARE_DOUBLE(analysis->analyse(settingsFor(Selection::TILES_INNER_DIAMOND)).backfocusMicrons, 0.0, 1e-6);

    // With optimised centres the weight of a tile follows its star offset: TL moves to (-6000, 4000) um
    f.offsets[TILE_TL] = QPoint(300, -300);
    auto shifted = makeAnalysis(f, 2.0);
    inject(*shifted, solutions);
    const double tlDistance = std::hypot(6000.0, 4000.0);
    const double shiftedMean = (1010.0 * tlDistance + 3.0 * 1010.0 * cornerDistance + 2.0 * 1000.0 * 5200.0
                                + 2.0 * 1000.0 * 7200.0) / (tlDistance + 3.0 * cornerDistance + 2.0 * 5200.0 + 2.0 * 7200.0);
    const Result on = shifted->analyse(settingsFor(Selection::TILES_ALL, true));
    QVERIFY(on.optCentres);
    COMPARE_DOUBLE(on.backfocusMicrons, (1000.0 - shiftedMean) * 2.0, 1e-6);
    QVERIFY(std::abs(on.backfocusMicrons - all.backfocusMicrons) > 1e-3);
    const Result off = shifted->analyse(settingsFor(Selection::TILES_ALL, false));
    COMPARE_DOUBLE(off.backfocusMicrons, all.backfocusMicrons, 1e-9);
}

void TestAberrationInspector::invalidSelection()
{
    const Fixture f = makeFixture(SCENARIO_B);
    auto analysis = makeAnalysis(f, 2.0);
    inject(*analysis, SCENARIO_B);

    // 3 is within the enumeration's value range but is not a selection (e.g. a combo box without a current item)
    const auto invalid = static_cast<Selection>(3);
    Settings settings;
    settings.tileSelection = invalid;
    const Result result = analysis->analyse(settings);
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        QVERIFY(!result.useTile[tile]);
        QVERIFY(!result.rows[tile].used);
    }
    QVERIFY(!result.backfocusOK);
    QVERIFY(!result.tiltOK);
    QVERIFY(!result.resultsOK);
    QCOMPARE(AberrationInspectorAnalysis::tileFromRow(invalid, 0), -1);
    QCOMPARE(AberrationInspectorAnalysis::tileSelectionToKey(invalid), QStringLiteral("outerCorners"));
}

QTEST_GUILESS_MAIN(TestAberrationInspector)
