/*
    SPDX-FileCopyrightText: 2023 John Evans <john.e.evans.email@googlemail.com>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#pragma once

#include "curvefit.h"
#include "aberrationinspectorutils.h"

#include <QJsonObject>
#include <QPoint>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QVariant>
#include <QVector2D>

#include <cstdint>
#include <functional>
#include <memory>

namespace Ekos
{

/**
 * @brief Parameters of an Aberration Inspector run, filled by Focus from the autofocus run that collected the data.
 *
 * This used to be AberrationInspector::abInsData. It lives here so that the headless analysis (and Focus) can use
 * it without pulling in the Qt Graphs headers of the dialog.
 */
struct AbInsData
{
    int run = 0;
    CurveFitting::CurveFit curveFit = CurveFitting::FOCUS_HYPERBOLA;
    bool useWeights = false;
    CurveFitting::OptimisationDirection optDir = CurveFitting::OPTIMISATION_MINIMISE;
    int sensorWidth = 0;            // pixels
    int sensorHeight = 0;           // pixels
    double pixelSize = 0.0;         // microns
    int tileWidth = 0;              // pixels (mosaic tiles are square)
    double focuserStepMicrons = 0.0;
    QString yAxisLabel;
    double starUnits = 1.0;
    double cfzSteps = 0.0;
    bool isPositionBased = true;
    // Optical train id, so the Tilt Correction Advisory can persist its settings
    // per train (0 = unknown → falls back to global settings).
    uint32_t opticalTrainID = 0;
};

/**
 * @class AberrationInspectorAnalysis
 * @brief Headless Aberration Inspector maths: per-tile curve fits, results table, backfocus and tilt.
 *
 * The Aberration Inspector dialog collects, for each of the 9 mosaic tiles, the focus measure at every focuser
 * position of an autofocus run. This class owns the analysis of that data so that it can run without a GUI (for
 * EkosLive clients and unit tests) while the dialog delegates to it and only renders the results.
 *
 * Usage: construct with the data collected by Focus, call fit() once, then analyse() with the user's settings
 * (tile selection, optimised tile centres, excluded tiles). analyse() can be called repeatedly with different
 * settings without refitting.
 *
 * The maths is deliberately identical to what the dialog did before the extraction (including the sign conventions
 * described on Result) so that the dialog and the headless result always agree.
 */
class AberrationInspectorAnalysis
{
    public:
        /** Which tiles to use in the analysis. Values match the abInsTileSelection combo box order. */
        enum TileSelection { TILES_ALL = 0, TILES_OUTER_CORNERS, TILES_INNER_DIAMOND };

        /** Backfocus adjustment advice, derived from the backfocus delta like the dialog's analysis text. */
        enum BackfocusAdvice { ADVICE_NONE = 0, ADVICE_NEARER, ADVICE_AWAY };

        /** Result of the curve fit of one tile. */
        struct TileFit
        {
            bool fit { false };         // curve fit found a solution
            int solution { 0 };         // focuser position of the solution (steps, rounded)
            double minMeasure { 0.0 };  // measure (e.g. HFR) at the solution
            double R2 { 0.0 };          // coefficient of determination of the fit
        };

        /** User settings that influence the analysis. */
        struct Settings
        {
            TileSelection tileSelection { TILES_OUTER_CORNERS };
            bool optCentres { false };
            bool exclude[NUM_TILES] { false, false, false, false, false, false, false, false, false };
        };

        /** One row of the results table. Deltas are tile − centre, like the dialog's table. */
        struct TileRow
        {
            int tile { 0 };
            bool used { false };
            bool fit { false };
            bool excluded { false };
            int solution { 0 };
            int deltaTicks { 0 };
            double deltaMicrons { 0.0 };   // deltaTicks * focuserStepMicrons, not truncated
            int minStars { 0 };
            int maxStars { 0 };
            double R2 { 0.0 };
            double minMeasure { 0.0 };
        };

        /**
         * Analysis result. Units: microns for backfocus, deltas, spans and tilt distances; % slope for tilt
         * percentages. deltasMicrons are centre − tile (the opposite sign of TileRow::deltaTicks), as in the dialog.
         */
        struct Result
        {
            TileSelection tileSelection { TILES_OUTER_CORNERS };
            bool optCentres { false };     // as applied (false when no tile offsets were available)
            bool useTile[NUM_TILES] { false, false, false, false, false, false, false, false, false };
            TileRow rows[NUM_TILES];
            bool backfocusOK { false };
            double backfocusMicrons { 0.0 };
            BackfocusAdvice advice { ADVICE_NONE };
            bool tiltOK { false };
            double deltasMicrons[NUM_TILES] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            double lrMicrons { 0.0 };
            double tbMicrons { 0.0 };
            double diagonalMicrons { 0.0 };
            double lrTiltPercent { 0.0 };
            double tbTiltPercent { 0.0 };
            double diagonalTiltPercent { 0.0 };
            double lrSpanMicrons { 0.0 };
            double tbSpanMicrons { 0.0 };
            bool resultsOK { false };      // backfocusOK && tiltOK
            QStringList warnings;          // insufficientDatapoints | datapointMismatch | tileOffsetsUnavailable
        };

        /**
         * Called by fit() after each tile, fitted or not. When the tile was fitted, curve is the solver, which still
         * holds that tile's solution so the caller can sample it (e.g. to plot it); fit.fit tells whether a solution
         * was found. When the tile was skipped (no data, too few datapoints) curve is nullptr.
         */
        using TileFitCallback = std::function<void(int tile, CurveFitting *curve, const TileFit &fit)>;

        /**
         * Minimum number of datapoints for a curve fit. The hyperbola has 4 parameters and the parabola 3; 4 keeps
         * the check independent of the curve type (autofocus runs always have at least 5 datapoints).
         */
        static constexpr int MIN_DATAPOINTS = 4;

        /**
         * Number of points at which fit() samples each fitted curve for toJson(), spread evenly over the desktop
         * plot's x range: the datapoint positions widened by 10 % of their span on each side.
         */
        static constexpr int CURVE_POINTS = 41;

        /**
         * @param data run parameters
         * @param positions focuser positions of the datapoints
         * @param measures per tile, the measure at each datapoint
         * @param weights per tile, the weight at each datapoint
         * @param numStars per tile, the number of stars at each datapoint
         * @param tileCenterOffsets per tile, the average star position relative to the tile centre (pixels)
         */
        AberrationInspectorAnalysis(const AbInsData &data, const QVector<int> &positions,
                                    const QVector<QVector<double>> &measures, const QVector<QVector<double >> &weights,
                                    const QVector<QVector<int>> &numStars, const QVector<QPoint> &tileCenterOffsets);

        /**
         * @brief Fit the curve of every tile. All tiles are fitted on one CurveFitting instance in tile order, as the
         * dialog always did (the solver warm-starts from the previous tile's solution).
         * @param onTileFitted optional callback invoked after each tile
         */
        void fit(const TileFitCallback &onTileFitted = TileFitCallback());

        /**
         * @brief Analyse the fitted tiles with the given settings: table rows, backfocus and tilt.
         */
        Result analyse(const Settings &settings) const;

        /** @brief Run parameters this analysis was built with. */
        const AbInsData &data() const
        {
            return m_data;
        }
        /** @brief Number of datapoints (focuser positions) in the run. */
        int datapointCount() const
        {
            return m_positions.count();
        }
        /** @brief Fit result of a tile (0 to NUM_TILES - 1; an invalid tile gives an unfit result). */
        const TileFit &tileFit(int tile) const;

        /** @brief Replace the fit of a tile. Used by tests and by the dialog's debug table editing. */
        void setTileFit(int tile, const TileFit &fit);
        /** @brief Replace the solution of a tile, keeping the rest of its fit. */
        void setSolution(int tile, int solution);

        /**
         * @brief Centre of a tile relative to the sensor centre, in microns (x right, y up).
         * @param optCentres add the average star offset of the tile to its geometric centre. The offsets are only
         * used when the run provided them for every tile.
         */
        QVector2D tileCentreMicrons(tileID tile, bool optCentres) const;

        /** @brief Fill useTile with the tiles of the given selection. */
        static void useTiles(TileSelection tileSelection, bool useTile[NUM_TILES]);
        /** @brief Tile shown in the given results table row, or -1 if the row is invalid. */
        static int tileFromRow(TileSelection tileSelection, int row);

        /**
         * @brief Tile selection stored in Options::abInsTileSelection, i.e. the (translated) combo box text.
         * Unknown text yields TILES_OUTER_CORNERS, the default.
         */
        static TileSelection tileSelectionFromOption(const QString &text);
        /** @brief The combo box text Options::abInsTileSelection stores for a selection. */
        static QString tileSelectionToOption(TileSelection tileSelection);
        /** @brief Tile selection from its machine-readable key: all | outerCorners | innerDiamond. */
        static TileSelection tileSelectionFromKey(const QString &key, bool *ok = nullptr);
        /** @brief Machine-readable key of a tile selection: all | outerCorners | innerDiamond. */
        static QString tileSelectionToKey(TileSelection tileSelection);
        /**
         * @brief Tile selection from a settings value: the machine-readable key or the combo box index (0 to 2).
         * @return false when the value is neither
         */
        static bool tileSelectionFromVariant(const QVariant &value, TileSelection *selection);
        /** @brief Machine-readable backfocus advice: none | nearer | away. */
        static QString adviceKey(BackfocusAdvice advice);
        /** @brief Machine-readable curve fit name: quadratic | hyperbola | parabola | gaussian2D | gaussian3D | plane. */
        static QString curveFitKey(CurveFitting::CurveFit curveFit);

        /**
         * @brief JSON representation of a result (camelCase keys, machine-readable codes, no translated text).
         *
         * Besides the result, it carries what a client needs to plot the run: the datapoint positions, each tile's
         * measures, weights and star counts as collected (their lengths may differ, see datapointMismatch), the
         * fitted curve sampled at curveX (empty for an unfit tile or before fit(); raw measures, 4 significant
         * digits) and the tile centre in microns from tileCentreMicrons() with the result's optCentres. Measures
         * and curves are in raw units: multiply by starUnits for the displayed value.
         *
         * Note: with optCentres the y of a tile centre inherits a sign quirk. Focus measures the star offset in
         * image coordinates (y down, focus.cpp) while tileCentreMicrons() adds it to a y-up centre, so the y offset
         * is applied mirrored. It is kept so that the result matches the desktop dialog.
         */
        QJsonObject toJson(const Result &result) const;

    private:
        bool calcBackfocusDelta(const Settings &settings, const Result &result, double &backfocusDelta) const;
        bool calcTilt(const Settings &settings, Result &result) const;
        bool avTiles(const int tiles[3], const Settings &settings, const Result &result, double &average) const;

        AbInsData m_data;
        QVector<int> m_positions;
        QVector<QVector<double>> m_measures;
        QVector<QVector<double>> m_weights;
        QVector<QVector<int>> m_numStars;
        QVector<QPoint> m_tileOffsets;

        std::unique_ptr<CurveFitting> m_curveFitting;
        TileFit m_tileFits[NUM_TILES];
        QStringList m_fitWarnings;
        // Fitted curves sampled by fit(): the positions, then each tile's raw measures (empty when not fit)
        QVector<double> m_curveX;
        QVector<double> m_curveY[NUM_TILES];
};

}
