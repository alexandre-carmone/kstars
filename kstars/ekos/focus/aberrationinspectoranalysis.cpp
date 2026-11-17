/*
    SPDX-FileCopyrightText: 2023 John Evans <john.e.evans.email@googlemail.com>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "aberrationinspectoranalysis.h"

#include <ekos_focus_debug.h>
#include <KLocalizedString>

#include <QJsonArray>
#include <QMetaType>

#include <algorithm>
#include <cmath>

namespace Ekos
{

AberrationInspectorAnalysis::AberrationInspectorAnalysis(const AbInsData &data, const QVector<int> &positions,
        const QVector<QVector<double>> &measures, const QVector<QVector<double >> &weights,
        const QVector<QVector<int>> &numStars, const QVector<QPoint> &tileCenterOffsets) :
    m_data(data), m_positions(positions), m_measures(measures), m_weights(weights),
    m_numStars(numStars), m_tileOffsets(tileCenterOffsets)
{
}

// Run curve fitting on the collected data for each tile. One CurveFitting instance is used for all tiles, in tile
// order, exactly as the dialog did: the solver seeds each fit from the previous solution.
void AberrationInspectorAnalysis::fit(const TileFitCallback &onTileFitted)
{
    m_curveFitting.reset(new CurveFitting());
    m_fitWarnings.clear();
    m_curveX.clear();
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        m_tileFits[tile] = TileFit();
        m_curveY[tile].clear();
    }

    const double expected = 0.0;
    int minPos = 0, maxPos = 0;

    QVector<bool> outliers;
    for (int i = 0; i < m_positions.count(); i++)
    {
        outliers.append(false);
        if (i == 0)
            minPos = maxPos = m_positions[i];
        else
        {
            minPos = std::min(minPos, m_positions[i]);
            maxPos = std::max(maxPos, m_positions[i]);
        }
    }

    // Positions at which the fitted curves are sampled: the desktop plot's x range (aberrationinspectorplot.cpp)
    if (maxPos > minPos)
    {
        const double border = (maxPos - minPos) / 10.0;
        const double lower = minPos - border;
        const double interval = (maxPos - minPos + 2.0 * border) / (CURVE_POINTS - 1);
        for (int i = 0; i < CURVE_POINTS; i++)
            m_curveX.append(lower + i * interval);
    }

    // The non-linear solver needs at least as many datapoints as curve parameters.
    const bool enoughPoints = m_positions.count() >= MIN_DATAPOINTS;
    if (!enoughPoints)
    {
        qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called with only %2 datapoints").arg(__FUNCTION__).arg(m_positions.count());
        m_fitWarnings.append(QStringLiteral("insufficientDatapoints"));
    }

    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        bool fitted = false;
        const bool haveData = tile < m_measures.count() && tile < m_weights.count()
                              && m_measures[tile].count() == m_positions.count()
                              && m_weights[tile].count() == m_positions.count();
        if (!haveData)
        {
            qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 datapoint mismatch for tile %2").arg(__FUNCTION__).arg(tile);
            if (!m_fitWarnings.contains(QStringLiteral("datapointMismatch")))
                m_fitWarnings.append(QStringLiteral("datapointMismatch"));
        }
        else if (enoughPoints)
        {
            fitted = true;
            m_curveFitting->fitCurve(CurveFitting::FittingGoal::BEST, m_positions, m_measures[tile], m_weights[tile], outliers,
                                     m_data.curveFit, m_data.useWeights, m_data.optDir);

            double position = 0.0;
            double measure = 0.0;
            double R2 = 0.0;
            const bool foundFit = m_curveFitting->findMinMax(expected, static_cast<double>(minPos), static_cast<double>(maxPos),
                                  &position, &measure, m_data.curveFit, m_data.optDir);
            if (foundFit)
                R2 = m_curveFitting->calculateR2(m_data.curveFit);

            m_tileFits[tile].fit = foundFit;
            m_tileFits[tile].solution = static_cast<int>(std::round(position));
            m_tileFits[tile].minMeasure = measure;
            m_tileFits[tile].R2 = R2;

            // Sample the curve while the solver still holds this tile's coefficients
            if (foundFit)
                for (double x : std::as_const(m_curveX))
                    m_curveY[tile].append(m_curveFitting->f(x));
        }

        if (onTileFitted)
            onTileFitted(tile, fitted ? m_curveFitting.get() : nullptr, m_tileFits[tile]);
    }
}

const AberrationInspectorAnalysis::TileFit & AberrationInspectorAnalysis::tileFit(int tile) const
{
    static const TileFit invalid;
    if (tile < 0 || tile >= NUM_TILES)
        return invalid;
    return m_tileFits[tile];
}

void AberrationInspectorAnalysis::setTileFit(int tile, const TileFit &fit)
{
    if (tile < 0 || tile >= NUM_TILES)
        return;
    m_tileFits[tile] = fit;
}

void AberrationInspectorAnalysis::setSolution(int tile, int solution)
{
    if (tile < 0 || tile >= NUM_TILES)
        return;
    m_tileFits[tile].solution = solution;
}

AberrationInspectorAnalysis::Result AberrationInspectorAnalysis::analyse(const Settings &settings) const
{
    Result result;
    result.tileSelection = settings.tileSelection;
    result.warnings = m_fitWarnings;

    // The tile centre offsets are only available once the autofocus run has reached its last datapoint
    result.optCentres = settings.optCentres;
    if (settings.optCentres && m_tileOffsets.count() < NUM_TILES)
    {
        result.optCentres = false;
        result.warnings.append(QStringLiteral("tileOffsetsUnavailable"));
    }

    useTiles(settings.tileSelection, result.useTile);

    // Results table
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        TileRow &row = result.rows[tile];
        row.tile = tile;
        row.used = result.useTile[tile];
        row.fit = m_tileFits[tile].fit;
        row.excluded = settings.exclude[tile];
        row.solution = m_tileFits[tile].solution;
        row.deltaTicks = m_tileFits[tile].solution - m_tileFits[TILE_CM].solution;
        row.deltaMicrons = row.deltaTicks * m_data.focuserStepMicrons;
        if (tile < m_numStars.count() && !m_numStars[tile].isEmpty())
        {
            row.minStars = *std::min_element(m_numStars[tile].constBegin(), m_numStars[tile].constEnd());
            row.maxStars = *std::max_element(m_numStars[tile].constBegin(), m_numStars[tile].constEnd());
        }
        row.R2 = m_tileFits[tile].R2;
        row.minMeasure = m_tileFits[tile].minMeasure;
    }

    // Sensor spans used for the tilt calculation: distance between the centres of opposite edge tiles
    result.lrSpanMicrons = (m_data.sensorWidth - m_data.tileWidth) * m_data.pixelSize;
    result.tbSpanMicrons = (m_data.sensorHeight - m_data.tileWidth) * m_data.pixelSize;

    // Backfocus
    // -result = move sensor nearer field flattener
    // +result = move sensor further from field flattener
    result.backfocusOK = calcBackfocusDelta(settings, result, result.backfocusMicrons);
    if (!result.backfocusOK)
        result.advice = ADVICE_NONE;
    else if (result.backfocusMicrons < -0.9)
        result.advice = ADVICE_NEARER;
    else if (result.backfocusMicrons > 0.9)
        result.advice = ADVICE_AWAY;
    else
        result.advice = ADVICE_NONE;

    // Tilt
    result.tiltOK = calcTilt(settings, result);

    result.resultsOK = result.backfocusOK && result.tiltOK;
    return result;
}

// Calculate the backfocus in microns
// Note that the tile positions are at different distances from the sensor centre so we need to weight
// values by the distance to tile centre. Also, we only include tiles for which curve fitting worked
// and for which the user hasn't elected to exclude
bool AberrationInspectorAnalysis::calcBackfocusDelta(const Settings &settings, const Result &result,
        double &backfocusDelta) const
{
    backfocusDelta = 0.0;

    auto usable = [&](int tile)
    {
        return m_tileFits[tile].fit && !settings.exclude[tile];
    };

    // Firstly check that we have a valid central tile - we can't do anything without that
    if (!usable(TILE_CM))
        return false;

    double dist = 0.0, sum = 0.0, counter = 0.0;

    switch (settings.tileSelection)
    {
        case TILES_ALL:
            // Use all usable tiles weighted by their distance from the centre
            for (int i = 0; i < NUM_TILES; i++)
            {
                if (i == TILE_CM)
                    continue;

                if (usable(i))
                {
                    dist = tileCentreMicrons(static_cast<tileID>(i), result.optCentres).length();
                    sum += m_tileFits[i].solution * dist;
                    counter += dist;
                }
            }
            break;

        case TILES_OUTER_CORNERS:
            // All tiles are diagonal from centre... so no need to weight the calc
            // Use tiles 0, 2, 6, 8
            for (int i :
                    {
                        TILE_TL, TILE_TR, TILE_BL, TILE_BR
                    })
            {
                if (usable(i))
                {
                    sum += m_tileFits[i].solution;
                    counter++;
                }
            }
            break;

        case TILES_INNER_DIAMOND:
            // Tiles are different distances from centre... so need to weight the calc
            // Use tiles 1, 3, 5, 7
            for (int i :
                    {
                        TILE_TM, TILE_CL, TILE_CR, TILE_BM
                    })
            {
                if (usable(i))
                {
                    dist = tileCentreMicrons(static_cast<tileID>(i), result.optCentres).length();
                    sum += m_tileFits[i].solution * dist;
                    counter += dist;
                }
            }
            break;

        default:
            qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called with invalid tile selection %2").arg(__FUNCTION__).arg(
                                           settings.tileSelection);
            return false;
    }

    if (counter == 0)
        // No valid tiles so can't complete the calc
        return false;

    backfocusDelta = (m_tileFits[TILE_CM].solution - (sum / counter)) * m_data.focuserStepMicrons;
    return true;
}

bool AberrationInspectorAnalysis::calcTilt(const Settings &settings, Result &result) const
{
    // Firstly check that we have a valid central tile - we can't do anything without that
    if (!m_tileFits[TILE_CM].fit || settings.exclude[TILE_CM])
        return false;

    // Calculate the deltas relative to the centre tile in microns
    for (int tile = 0; tile < NUM_TILES; tile++)
        result.deltasMicrons[tile] = (m_tileFits[TILE_CM].solution - m_tileFits[tile].solution) * m_data.focuserStepMicrons;

    // Calculate the average tile position for left, right, top and bottom. If any of these cannot be
    // calculated then fail the whole calculation.
    double avLeft, avRight, avTop, avBottom;
    const int tilesL[3] = { TILE_TL, TILE_CL, TILE_BL };
    if (!avTiles(tilesL, settings, result, avLeft))
        return false;
    const int tilesR[3] = { TILE_TR, TILE_CR, TILE_BR };
    if (!avTiles(tilesR, settings, result, avRight))
        return false;
    const int tilesT[3] = { TILE_TL, TILE_TM, TILE_TR };
    if (!avTiles(tilesT, settings, result, avTop))
        return false;
    const int tilesB[3] = { TILE_BL, TILE_BM, TILE_BR };
    if (!avTiles(tilesB, settings, result, avBottom))
        return false;

    result.lrMicrons = avLeft - avRight;
    result.tbMicrons = avTop - avBottom;
    result.diagonalMicrons = std::hypot(result.lrMicrons, result.tbMicrons);

    // Calculate the tilt as a % slope
    result.lrTiltPercent = result.lrMicrons / result.lrSpanMicrons * 100.0;
    result.tbTiltPercent = result.tbMicrons / result.tbSpanMicrons * 100.0;
    result.diagonalTiltPercent = std::hypot(result.lrTiltPercent, result.tbTiltPercent);
    return true;
}

// Averages upto 3 passed in tile values.
bool AberrationInspectorAnalysis::avTiles(const int tiles[3], const Settings &settings, const Result &result,
        double &average) const
{
    double sum = 0.0;
    int counter = 0;
    for (int i = 0; i < 3; i++)
    {
        const int tile = tiles[i];
        if (result.useTile[tile] && m_tileFits[tile].fit && !settings.exclude[tile])
        {
            sum += result.deltasMicrons[tile];
            counter++;
        }
    }
    if (counter > 0)
        average = sum / counter;
    return (counter > 0);
}

// Returns the X, Y centre of the tile in microns
QVector2D AberrationInspectorAnalysis::tileCentreMicrons(tileID tile, bool optCentres) const
{
    const double halfSW = m_data.sensorWidth * m_data.pixelSize / 2.0;
    const double halfSH = m_data.sensorHeight * m_data.pixelSize / 2.0;
    const double halfTS = m_data.tileWidth * m_data.pixelSize / 2.0;

    // Focus calculates the average star position in each tile and passes this to Aberration Inspector as
    // an x, y offset from the center of the tile. If stars are homogeneously distributed then the offset would
    // be 0, 0. If they aren't, then offset represents how much to add to the tile centre.
    // A user option (abInsOptCentres) specifies whether to use the offsets. Focus provides the offsets for all
    // tiles or not at all, so a partial set is not used.
    double xOffset = 0.0;
    double yOffset = 0.0;
    if (optCentres && m_tileOffsets.count() >= NUM_TILES && tile >= 0 && tile < NUM_TILES)
    {
        xOffset = m_tileOffsets[tile].x() * m_data.pixelSize;
        yOffset = m_tileOffsets[tile].y() * m_data.pixelSize;
    }

    switch (tile)
    {
        case TILE_TL:
            return QVector2D(-(halfSW - halfTS) + xOffset, halfSH - halfTS + yOffset);

        case TILE_TM:
            return QVector2D(xOffset, halfSH - halfTS + yOffset);

        case TILE_TR:
            return QVector2D(halfSW - halfTS + xOffset, halfSH - halfTS + yOffset);

        case TILE_CL:
            return QVector2D(-(halfSW - halfTS) + xOffset, yOffset);

        case TILE_CM:
            return QVector2D(xOffset, yOffset);

        case TILE_CR:
            return QVector2D(halfSW - halfTS + xOffset, yOffset);

        case TILE_BL:
            return QVector2D(-(halfSW - halfTS) + xOffset, -(halfSH - halfTS) + yOffset);

        case TILE_BM:
            return QVector2D(xOffset, -(halfSH - halfTS) + yOffset);

        case TILE_BR:
            return QVector2D(halfSW - halfTS + xOffset, -(halfSH - halfTS) + yOffset);

        default:
            qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called with invalid tile %2").arg(__FUNCTION__).arg(tile);
            return QVector2D(0.0, 0.0);
    }
}

void AberrationInspectorAnalysis::useTiles(TileSelection tileSelection, bool useTile[NUM_TILES])
{
    switch (tileSelection)
    {
        case TILES_ALL:
            // Use all tiles
            for (int i = 0; i < NUM_TILES; i++)
                useTile[i] = true;
            break;

        case TILES_OUTER_CORNERS:
            // Use tiles TL, TR, CM, BL, BR
            useTile[TILE_TL] = useTile[TILE_TR] = useTile[TILE_CM] = useTile[TILE_BL] = useTile[TILE_BR] = true;
            useTile[TILE_TM] = useTile[TILE_CL] = useTile[TILE_CR] = useTile[TILE_BM] = false;
            break;

        case TILES_INNER_DIAMOND:
            // Use tiles TM, CL, CM, CR, BM
            useTile[TILE_TL] = useTile[TILE_TR] = useTile[TILE_BL] = useTile[TILE_BR] = false;
            useTile[TILE_TM] = useTile[TILE_CL] = useTile[TILE_CM] = useTile[TILE_CR] = useTile[TILE_BM] = true;
            break;

        default:
            qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called with invalid tile selection %2").arg(__FUNCTION__).arg(tileSelection);
            break;
    }
}

int AberrationInspectorAnalysis::tileFromRow(TileSelection tileSelection, int row)
{
    int tile = -1;

    if (row < 0 || row >= NUM_TILES)
    {
        qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called with invalid row %2").arg(__FUNCTION__).arg(row);
        return tile;
    }

    switch (tileSelection)
    {
        case TILES_ALL:
            // Use all tiles
            tile = row;
            break;

        case TILES_OUTER_CORNERS:
            // Use tiles 0, 2, 4, 6, 8
            tile = row * 2;
            break;

        case TILES_INNER_DIAMOND:
            // Use tiles 1, 3, 4, 5, 7
            if (row < 2)
                tile = (row * 2) + 1;
            else if (row == 2)
                tile = 4;
            else
                tile = (row * 2) - 1;
            break;

        default:
            qCDebug(KSTARS_EKOS_FOCUS) << QString("%1 called but TileSelection invalid").arg(__FUNCTION__);
            break;
    }
    return tile;
}

// Options::abInsTileSelection stores the text of the abInsTileSelection combo box, which is translated. Accept the
// translated texts (what the dialog saves) as well as the English originals (the kcfg default).
AberrationInspectorAnalysis::TileSelection AberrationInspectorAnalysis::tileSelectionFromOption(const QString &text)
{
    static const QString english[] = { QStringLiteral("All"), QStringLiteral("Centre and outer corners"),
                                       QStringLiteral("Centre and inner diamond")
                                     };
    for (int i = TILES_ALL; i <= TILES_INNER_DIAMOND; i++)
    {
        const auto selection = static_cast<TileSelection>(i);
        if (text == tileSelectionToOption(selection) || text == english[i])
            return selection;
    }
    return TILES_OUTER_CORNERS;
}

QString AberrationInspectorAnalysis::tileSelectionToOption(TileSelection tileSelection)
{
    // These must match the items of the abInsTileSelection combo box in aberrationinspector.ui
    switch (tileSelection)
    {
        case TILES_ALL:
            return i18n("All");
        case TILES_INNER_DIAMOND:
            return i18n("Centre and inner diamond");
        case TILES_OUTER_CORNERS:
        default:
            return i18n("Centre and outer corners");
    }
}

AberrationInspectorAnalysis::TileSelection AberrationInspectorAnalysis::tileSelectionFromKey(const QString &key,
        bool *ok)
{
    for (int i = TILES_ALL; i <= TILES_INNER_DIAMOND; i++)
    {
        const auto selection = static_cast<TileSelection>(i);
        if (key == tileSelectionToKey(selection))
        {
            if (ok)
                *ok = true;
            return selection;
        }
    }
    if (ok)
        *ok = false;
    return TILES_OUTER_CORNERS;
}

QString AberrationInspectorAnalysis::tileSelectionToKey(TileSelection tileSelection)
{
    switch (tileSelection)
    {
        case TILES_ALL:
            return QStringLiteral("all");
        case TILES_INNER_DIAMOND:
            return QStringLiteral("innerDiamond");
        case TILES_OUTER_CORNERS:
        default:
            return QStringLiteral("outerCorners");
    }
}

bool AberrationInspectorAnalysis::tileSelectionFromVariant(const QVariant &value, TileSelection *selection)
{
    if (value.userType() == QMetaType::Bool)
        return false;
    bool ok = false;
    const int index = value.toInt(&ok);
    if (ok)
    {
        if (index < TILES_ALL || index > TILES_INNER_DIAMOND)
            return false;
        if (selection)
            *selection = static_cast<TileSelection>(index);
        return true;
    }
    const TileSelection fromKey = tileSelectionFromKey(value.toString(), &ok);
    if (ok && selection)
        *selection = fromKey;
    return ok;
}

QString AberrationInspectorAnalysis::adviceKey(BackfocusAdvice advice)
{
    switch (advice)
    {
        case ADVICE_NEARER:
            return QStringLiteral("nearer");
        case ADVICE_AWAY:
            return QStringLiteral("away");
        case ADVICE_NONE:
        default:
            return QStringLiteral("none");
    }
}

QString AberrationInspectorAnalysis::curveFitKey(CurveFitting::CurveFit curveFit)
{
    switch (curveFit)
    {
        case CurveFitting::FOCUS_QUADRATIC:
            return QStringLiteral("quadratic");
        case CurveFitting::FOCUS_HYPERBOLA:
            return QStringLiteral("hyperbola");
        case CurveFitting::FOCUS_PARABOLA:
            return QStringLiteral("parabola");
        case CurveFitting::FOCUS_2DGAUSSIAN:
            return QStringLiteral("gaussian2D");
        case CurveFitting::FOCUS_3DGAUSSIAN:
            return QStringLiteral("gaussian3D");
        case CurveFitting::FOCUS_PLANE:
            return QStringLiteral("plane");
    }
    return QStringLiteral("unknown");
}

namespace
{
// Rounds to the given number of significant digits, to keep the sampled curves short on the wire
double roundSignificant(double value, int digits)
{
    if (value == 0.0 || !std::isfinite(value))
        return value;
    const double scale = std::pow(10.0, digits - 1 - std::floor(std::log10(std::abs(value))));
    return std::round(value * scale) / scale;
}
}

QJsonObject AberrationInspectorAnalysis::toJson(const Result &result) const
{
    QJsonObject json;
    json.insert(QStringLiteral("run"), m_data.run);
    json.insert(QStringLiteral("curveFit"), curveFitKey(m_data.curveFit));
    json.insert(QStringLiteral("useWeights"), m_data.useWeights);
    json.insert(QStringLiteral("datapoints"), m_positions.count());
    json.insert(QStringLiteral("yAxisLabel"), m_data.yAxisLabel);
    json.insert(QStringLiteral("starUnits"), m_data.starUnits);
    json.insert(QStringLiteral("focuserStepMicrons"), m_data.focuserStepMicrons);
    json.insert(QStringLiteral("cfzSteps"), m_data.cfzSteps);

    QJsonObject sensor;
    sensor.insert(QStringLiteral("widthPx"), m_data.sensorWidth);
    sensor.insert(QStringLiteral("heightPx"), m_data.sensorHeight);
    sensor.insert(QStringLiteral("pixelSizeMicrons"), m_data.pixelSize);
    sensor.insert(QStringLiteral("tileWidthPx"), m_data.tileWidth);
    sensor.insert(QStringLiteral("lrSpanMicrons"), result.lrSpanMicrons);
    sensor.insert(QStringLiteral("tbSpanMicrons"), result.tbSpanMicrons);
    json.insert(QStringLiteral("sensor"), sensor);

    json.insert(QStringLiteral("tileSelection"), tileSelectionToKey(result.tileSelection));
    json.insert(QStringLiteral("optCentres"), result.optCentres);
    json.insert(QStringLiteral("warnings"), QJsonArray::fromStringList(result.warnings));
    json.insert(QStringLiteral("optDir"), m_data.optDir == CurveFitting::OPTIMISATION_MAXIMISE ?
                QStringLiteral("maximise") : QStringLiteral("minimise"));

    QJsonArray positions;
    for (int position : m_positions)
        positions.append(position);
    json.insert(QStringLiteral("positions"), positions);
    QJsonArray curveX;
    for (double x : m_curveX)
        curveX.append(x);
    json.insert(QStringLiteral("curveX"), curveX);

    QJsonArray tiles;
    for (int tile = 0; tile < NUM_TILES; tile++)
    {
        const TileRow &row = result.rows[tile];
        QJsonObject item;
        item.insert(QStringLiteral("id"), row.tile);
        item.insert(QStringLiteral("name"), TILE_NAME[tile]);
        item.insert(QStringLiteral("used"), row.used);
        item.insert(QStringLiteral("fit"), row.fit);
        item.insert(QStringLiteral("excluded"), row.excluded);
        item.insert(QStringLiteral("solution"), row.solution);
        item.insert(QStringLiteral("deltaTicks"), row.deltaTicks);
        item.insert(QStringLiteral("deltaMicrons"), row.deltaMicrons);
        item.insert(QStringLiteral("minStars"), row.minStars);
        item.insert(QStringLiteral("maxStars"), row.maxStars);
        item.insert(QStringLiteral("r2"), row.R2);
        item.insert(QStringLiteral("minMeasure"), row.minMeasure);

        QJsonArray measures, weights, numStars, curve;
        if (tile < m_measures.count())
            for (double measure : m_measures[tile])
                measures.append(measure);
        if (tile < m_weights.count())
            for (double weight : m_weights[tile])
                weights.append(weight);
        if (tile < m_numStars.count())
            for (int stars : m_numStars[tile])
                numStars.append(stars);
        for (double y : m_curveY[tile])
            curve.append(roundSignificant(y, 4));
        item.insert(QStringLiteral("measures"), measures);
        item.insert(QStringLiteral("weights"), weights);
        item.insert(QStringLiteral("numStars"), numStars);
        item.insert(QStringLiteral("curve"), curve);

        const QVector2D centre = tileCentreMicrons(static_cast<tileID>(tile), result.optCentres);
        item.insert(QStringLiteral("centreXMicrons"), centre.x());
        item.insert(QStringLiteral("centreYMicrons"), centre.y());
        tiles.append(item);
    }
    json.insert(QStringLiteral("tiles"), tiles);

    QJsonObject backfocus;
    backfocus.insert(QStringLiteral("ok"), result.backfocusOK);
    backfocus.insert(QStringLiteral("deltaMicrons"), result.backfocusMicrons);
    backfocus.insert(QStringLiteral("advice"), adviceKey(result.advice));
    json.insert(QStringLiteral("backfocus"), backfocus);

    QJsonObject tilt;
    tilt.insert(QStringLiteral("ok"), result.tiltOK);
    tilt.insert(QStringLiteral("lrMicrons"), result.lrMicrons);
    tilt.insert(QStringLiteral("tbMicrons"), result.tbMicrons);
    tilt.insert(QStringLiteral("diagonalMicrons"), result.diagonalMicrons);
    tilt.insert(QStringLiteral("lrPercent"), result.lrTiltPercent);
    tilt.insert(QStringLiteral("tbPercent"), result.tbTiltPercent);
    tilt.insert(QStringLiteral("diagonalPercent"), result.diagonalTiltPercent);
    json.insert(QStringLiteral("tilt"), tilt);

    json.insert(QStringLiteral("resultsOK"), result.resultsOK);
    return json;
}

}
