#include <QCoreApplication>
#include <QLocale>
#include <QTranslator>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include "traindefinition/train.h"
#include "traindefinition/trainslist.h"
#include "network/network.h"
#include "simulator.h"
#include "util/vector.h"
#include <iostream>
#include <sstream>
#include <QCoreApplication>
#include <QCommandLineParser>
#include <stdio.h>
#include <filesystem>
#include "qdir.h"
#include "simulatorapi.h"
#include "errorhandler.h"
#include "VersionConfig.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

const std::string compilation_date = __DATE__;
const std::string compilation_time = __TIME__;
const QString NETWORK_NAME = "mainNetwork";
/** The distances ahead of the train (m) reported in interactive mode */
const std::vector<double> LOOKAHEAD_DISTANCES = {500.0, 1000.0, 2000.0};
/**
 * @brief checkParserValue
 * @param parser
 * @param option
 * @return
 */
bool checkParserValue(QCommandLineParser& parser, const QCommandLineOption &option, std::string s, bool isRequired = true){
    if(parser.isSet(option)) {
        return true;
    }
    if (isRequired){
        fputs(qPrintable(QString::fromStdString(s)), stdout);
        fputs("\n\n", stdout);
        fputs(qPrintable(parser.helpText()), stdout);
    }
    return false;
}

/**
 * @brief Parses one interactive command and applies it to all trains.
 *
 * A command is one JSON object per line with exactly one of:
 *  {"target_speed_mps": 15.0}  follow min(speed limit, 15 m/s)
 *  {"target_frac": 0.7}        follow 70% of the current speed limit
 *  {"coast": true}             no traction, the train rolls freely
 *
 * @param line  The command line read from stdin.
 * @param trains The trains to control.
 */
void applyInteractiveCommand(const std::string &line,
                             const QVector<std::shared_ptr<Train>> &trains) {
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(line), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        throw std::runtime_error("Interactive input must be a JSON object, e.g. "
                                 "{\"target_frac\": 0.7}, {\"target_speed_mps\": 15.0} "
                                 "or {\"coast\": true}. Got: " + line);
    }
    QJsonObject cmd = doc.object();
    bool hasSpeed = cmd.contains("target_speed_mps");
    bool hasFraction = cmd.contains("target_frac");
    bool coast = cmd.value("coast").toBool(false);
    if (int(hasSpeed) + int(hasFraction) + int(coast) != 1) {
        throw std::runtime_error("Provide exactly one of 'target_speed_mps', "
                                 "'target_frac' or 'coast': true. Got: " + line);
    }
    double targetSpeed = cmd.value("target_speed_mps").toDouble(0.0);
    double targetFraction = cmd.value("target_frac").toDouble(0.0);
    if (hasSpeed && targetSpeed <= 0.0) {
        throw std::runtime_error("'target_speed_mps' must be a positive number. Got: " + line);
    }
    if (hasFraction && (targetFraction <= 0.0 || targetFraction > 1.0)) {
        throw std::runtime_error("'target_frac' must be in (0, 1]. Got: " + line);
    }

    for (auto &t : trains) {
        // the agent drives the train, so the built-in optimizer stays off
        t->optimize = false;
        t->rlSpeedControl = true;
        t->rlCoast = coast;
        t->rlTargetSpeed = hasSpeed ? targetSpeed : 0.0;
        t->rlTargetFraction = hasFraction ? targetFraction : 0.0;
    }
}

/**
 * @brief Mean grade and lowest speed limit over each look-ahead distance
 * from the train's tip, following the train's path.
 *
 * If the path ends inside a distance, its lowest speed limit is 0 because the
 * train has to stop at the end of its path.
 */
void getLookAhead(Network *net, const std::shared_ptr<Train> &train,
                  std::vector<double> &meanGrades, std::vector<double> &minSpeedLimits) {
    size_t n = LOOKAHEAD_DISTANCES.size();
    std::vector<double> covered(n, 0.0);
    meanGrades.assign(n, 0.0);
    minSpeedLimits.assign(n, std::numeric_limits<double>::max());

    double from = train->travelledDistance;
    double farthest = from + *std::max_element(LOOKAHEAD_DISTANCES.begin(), LOOKAHEAD_DISTANCES.end());
    int firstIndex = std::max(0, int(train->trainPath.index(train->previousNodeID)));
    for (int i = firstIndex; i + 1 < int(train->linksCumLengths.size()); i++) {
        double linkStart = train->linksCumLengths[i];
        double linkEnd = train->linksCumLengths[i + 1];
        if (linkEnd <= from) { continue; }
        if (linkStart >= farthest) { break; }
        std::shared_ptr<NetLink> link =
            net->getLinkByStartandEndNodeID(train, train->trainPath[i], train->trainPath[i + 1], false);
        // the grade in the direction of travel is stored by the link's first node on the path
        auto grade = link->grade.find(train->trainPath[i]);
        double gradeValue = (grade != link->grade.end()) ? grade->second : 0.0;
        for (size_t h = 0; h < n; h++) {
            double overlap = std::min(linkEnd, from + LOOKAHEAD_DISTANCES[h]) - std::max(linkStart, from);
            if (overlap <= 0.0) { continue; }
            meanGrades[h] += gradeValue * overlap;
            covered[h] += overlap;
            minSpeedLimits[h] = std::min(minSpeedLimits[h], link->freeFlowSpeed);
        }
    }
    for (size_t h = 0; h < n; h++) {
        meanGrades[h] = (covered[h] > 0.0) ? meanGrades[h] / covered[h] : 0.0;
        // the path ends within this distance
        if (covered[h] < LOOKAHEAD_DISTANCES[h] - 1e-6) { minSpeedLimits[h] = 0.0; }
    }
}

/**
 * @brief Writes the state of the first train as one line on stdout:
 * "NTS_JSON {...}".
 */
void writeInteractiveState(Simulator *sim, Network *net,
                           const QVector<std::shared_ptr<Train>> &trains) {
    QJsonObject state;
    state["timestep"] = sim->getCurrentStateAsJson()["CurrentSimulationTime"].toDouble();

    std::shared_ptr<Train> train = trains.empty() ? nullptr : trains.front();
    bool started = train != nullptr && train->loaded && train->currentFirstLink != nullptr;

    double grade = 0.0, curvature = 0.0, linkMaxSpeed = 0.0;
    std::vector<double> meanGrades(LOOKAHEAD_DISTANCES.size(), 0.0);
    std::vector<double> minSpeedLimits(LOOKAHEAD_DISTANCES.size(), 0.0);
    if (started) {
        std::shared_ptr<NetLink> link = train->currentFirstLink;
        if (train->LinkGradeDirection.count(link->id) > 0) { grade = train->LinkGradeDirection[link->id]; }
        curvature = link->curvature;
        linkMaxSpeed = link->freeFlowSpeed;
        getLookAhead(net, train, meanGrades, minSpeedLimits);
    }

    QJsonArray distances, gradesAhead, limitsAhead;
    for (size_t h = 0; h < LOOKAHEAD_DISTANCES.size(); h++) {
        distances.append(LOOKAHEAD_DISTANCES[h]);
        gradesAhead.append(meanGrades[h]);
        limitsAhead.append(minSpeedLimits[h]);
    }

    state["speed_mps"] = started ? train->currentSpeed : 0.0;
    state["acceleration_mps2"] = started ? train->currentAcceleration : 0.0;
    state["position_m"] = started ? train->travelledDistance : 0.0;
    state["remaining_dist_m"] = started ?
        std::max(0.0, train->trainTotalPathLength - train->travelledDistance) : 0.0;
    state["grade_perc"] = grade;
    state["curvature_perc"] = curvature;
    // energy used in the last step (kWh), net of regeneration
    state["energy_kwh"] = started ? train->energyStat : 0.0;
    // trip totals so far (kWh): net = consumed - regenerated
    state["energy_consumed_kwh_cum"] = started ? train->totalEConsumed : 0.0;
    state["energy_regenerated_kwh_cum"] = started ? train->totalERegenerated : 0.0;
    state["energy_net_kwh_cum"] = started ? train->cumEnergyStat : 0.0;
    state["link_max_speed_mps"] = linkMaxSpeed;
    state["speed_limit_mps"] = started ? train->rlSpeedLimit : 0.0;
    state["applied_cap_mps"] = started ? train->rlAppliedCap : 0.0;
    state["coasting"] = started && train->rlCoast;
    // the notch the simulator chose for the speed it drove at. reported for
    // information only, the agent does not set the notch
    bool hasLoco = train != nullptr && !train->locomotives.empty();
    state["notch"] = (started && hasLoco) ? train->locomotives.front()->currentLocNotch : 0;
    state["notch_max"] = hasLoco ? train->locomotives.front()->Nmax : 8;
    state["lookahead_m"] = distances;
    state["grade_ahead_perc"] = gradesAhead;
    state["speed_limit_min_ahead_mps"] = limitsAhead;
    state["terminated"] = train != nullptr && train->reachedDestination;

    std::cout << "NTS_JSON " << QJsonDocument(state).toJson(QJsonDocument::Compact).constData() << std::endl;
}

/**
 * @brief Runs the simulator one time step per command read from stdin and
 * writes the train state after each step, until all trains arrive or stdin
 * closes.
 */
void runInteractive(Simulator *sim) {
    sim->initializeSimulator(false);
    std::cout << "Starting the Simulator in interactive mode.\n";
    std::cout.flush();

    Network *net = SimulatorAPI::ContinuousMode::getNetwork(NETWORK_NAME);
    QVector<std::shared_ptr<Train>> trains = SimulatorAPI::ContinuousMode::getTrains(NETWORK_NAME);

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) { continue; }
        applyInteractiveCommand(line, trains);
        sim->runOneTimeStep();
        writeInteractiveState(sim, net, trains);
        QCoreApplication::processEvents();

        bool allArrived = std::all_of(trains.begin(), trains.end(),
                                      [](const std::shared_ptr<Train> &t) { return t->reachedDestination; });
        if (allArrived) { break; }
    }

    // write the summary file and close the trajectory file. these are called
    // directly: the simulationFinished signal would queue them to the
    // simulator's worker thread, where they race the end of the program.
    sim->generateSummaryData();
    sim->exportSummaryToTXTFile();
    sim->finalizeSimulation();
    QCoreApplication::processEvents();
}

/**
 * Main entry-point for this application
 *
 * @author	Ahmed Aredah
 * @date	2/28/2023
 *
 * @param 	argc	The number of command-line arguments provided.
 * @param 	argv	An array of command-line argument strings.
 *
 * @returns	Exit-code for the process - 0 for success, else an error code.
 */
int main(int argc, char *argv[])
{
    // ####################################################
    // #                     values                       #
    // ####################################################
    std::string GithubLink = "https://github.com/VTTI-CSM/NeTrainSim";

    QCoreApplication app(argc, argv);
    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "NeTrainSim_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            app.installTranslator(&translator);
            break;
        }
    }
    QCoreApplication::setApplicationName(NeTrainSim_NAME);
    QCoreApplication::setApplicationVersion(NeTrainSim_VERSION);
    QCoreApplication::setOrganizationName(NeTrainSim_VENDOR);

    QCommandLineParser parser;
    parser.setApplicationDescription("Open-source network train simulator");
    QCommandLineOption helpOption(QStringList() << "h" << "help" << "?",
                                  "Display this help message.");
    parser.addOption(helpOption);
    //parser.addHelpOption();
    parser.addVersionOption();


    const QCommandLineOption nodesOption(QStringList() << "n" << "nodes",
                                         QCoreApplication::translate("main", "[Required] the nodes filename."), "nodesFile", "");
    parser.addOption(nodesOption);

    const QCommandLineOption linksOption(QStringList() << "l" << "links",
                                         QCoreApplication::translate("main", "[Required] the links filename."), "linksFile", "");
    parser.addOption(linksOption);

    const QCommandLineOption trainsOption(QStringList() << "t" << "trains",
                                          QCoreApplication::translate("main", "[Required] the trains filename."), "trainsFile", "");
    parser.addOption(trainsOption);

    const QCommandLineOption outputLocationOption(QStringList() << "o" << "output",
                                                  QCoreApplication::translate("main", "[Optional] the output folder address. \nDefault is 'C:\\Users\\<USERNAME>\\Documents\\NeTrainSim\\'."), "outputLocation", "");
    parser.addOption(outputLocationOption);

    const QCommandLineOption summaryFilenameOption(QStringList() << "s" << "summary",
                                                   QCoreApplication::translate("main", "[Optional] the summary filename. \nDefault is 'trainSummary_timeStamp.txt'."), "summaryFilename", "");
    parser.addOption(summaryFilenameOption);

    const QCommandLineOption summaryExportAllOption(QStringList() << "a" << "all",
                                                    QCoreApplication::translate("main", "[Optional] bool to show summary of all trains in the summary file. \nDefault is 'false'."), "summarizeAllTrains", "false");
    parser.addOption(summaryExportAllOption);

    const QCommandLineOption exportInstaTrajOption(QStringList() << "e" << "export",
                                                   QCoreApplication::translate("main", "[Optional] bool to export instantaneous trajectory. \nDefault is 'false'."), "exportTrajectoryOptions" ,"false");
    parser.addOption(exportInstaTrajOption);

    const QCommandLineOption instaTrajOption(QStringList() << "i" << "insta",
                                             QCoreApplication::translate("main", "[Optional] the instantaneous trajectory filename. \nDefault is 'trainTrajectory_timeStamp.csv'."), "instaTrajectoryFile", "");
    parser.addOption(instaTrajOption);

    const QCommandLineOption timeStepOption(QStringList() << "p" << "timeStep",
                                            QCoreApplication::translate("main", "[Optional] the simulator time step. \nDefault is '1.0'."), "simulatorTimeStep", "1.0");
    parser.addOption(timeStepOption);

    const QCommandLineOption enableOptimization(QStringList() << "z" << "optimization",
                                                QCoreApplication::translate("main", "[Optional] bool to enable single train trajectory optimization. \nDefault is 'false'"), "optimization", "false");
    parser.addOption(enableOptimization);

    const QCommandLineOption optimizationEvery(QStringList() << "y" << "optimizeEvery",
                                               QCoreApplication::translate("main", "[Optional] the optimization frequency. \nDefault is '1'."), "optimizeEvery", "1");
    parser.addOption(optimizationEvery);

    const QCommandLineOption optimizationLookahead(QStringList() << "d" << "optimizerLookahead",
                                                   QCoreApplication::translate("main", "[Optional] the forward lookahead distance for the optimizer. \nDefault is '1'."), "optimizerLookahead", "1");
    parser.addOption(optimizationLookahead);

    const QCommandLineOption optimizationSpeedPriorityFactor(QStringList() << "k" << "OptimizationSpeedFactor",
                                                             QCoreApplication::translate("main", "[Optional] the speed priority factor in case of optimization. \n Default is '0.0'."), "OptimizationSpeedFactor", "0.0");
    parser.addOption(optimizationSpeedPriorityFactor);

    const QCommandLineOption interactiveOption(QStringList() << "I" << "interactive",
                                               QCoreApplication::translate("main", "[Optional] run in interactive mode: read one speed command per line from stdin as JSON ({\"target_frac\": 0.7}, {\"target_speed_mps\": 15.0} or {\"coast\": true}), run one time step, and write the train state as a 'NTS_JSON {...}' line on stdout. \nDefault is off."));
    parser.addOption(interactiveOption);

    // process all the arguments
    parser.process(app);

    // display the help if requested and exit
    if (parser.isSet(helpOption)) {
        parser.showHelp(0);
    }

    // show app details
    std::stringstream hellos;
    hellos << NeTrainSim_NAME << " [Version " << NeTrainSim_VERSION << ", "  << compilation_date << " " << compilation_time << " Build" <<  "]" << endl;
    hellos << NeTrainSim_VENDOR << endl;
    hellos << GithubLink << endl;
    std::cout << hellos.str() << "\n";




    std::string nodesFile, linksFile, trainsFile, exportLocation, summaryFilename, instaTrajFilename;
    bool exportInstaTraj = false;
    double timeStep = 1.0;
    bool optimize = false;
    double optimize_speedfactor = 0.0;
    int optimizerFrequency = 0;
    int lookahead = 0;

    // read values from the cmd
    // read required values

    if (checkParserValue(parser, nodesOption, "nodes file is missing!", true)) { nodesFile = parser.value(nodesOption).toStdString(); }
    else { return 1;}
    if (checkParserValue(parser, linksOption, "links file is missing!", true)) { linksFile = parser.value(linksOption).toStdString(); }
    else { return 1;}
    if (checkParserValue(parser, trainsOption, "trains file is missing!", true)) { trainsFile = parser.value(trainsOption).toStdString(); }
    else { return 1;}

    // read optional values
    if (checkParserValue(parser, outputLocationOption, "" ,false)){
        exportLocation = parser.value(outputLocationOption).toStdString();
        QDir directory(QString::fromStdString(exportLocation));
        // check if directory is valid
        if (!directory.exists()) {
            fputs(qPrintable("export directory is not valid!"), stdout);
            return 1;
        }
    }
    else { exportLocation = ""; }

    if (checkParserValue(parser, summaryFilenameOption, "", false)){ summaryFilename = parser.value(summaryFilenameOption).toStdString(); }
    else { summaryFilename = ""; }

    if (checkParserValue(parser, exportInstaTrajOption, "", false)){
        stringstream ss(parser.value(exportInstaTrajOption).toStdString());
        ss >> std::boolalpha >> exportInstaTraj;
    }
    else { exportInstaTraj = false; }

    if (checkParserValue(parser, instaTrajOption, "", false)){ instaTrajFilename = parser.value(instaTrajOption).toStdString(); }
    else { instaTrajFilename = ""; }

    if (checkParserValue(parser, timeStepOption, "", false)) { timeStep = parser.value(timeStepOption).toDouble(); }
    else { timeStep = 1.0; }

    if (checkParserValue(parser, enableOptimization, "", false)){
        stringstream ss(parser.value(enableOptimization).toStdString());
        ss >> std::boolalpha >> optimize;
    }
    else { optimize = false; }

    if (checkParserValue(parser, optimizationEvery, "", false)) { optimizerFrequency = parser.value(optimizationEvery).toInt(); }
    else { optimizerFrequency = 1.0; }
    if (checkParserValue(parser, optimizationLookahead, "", false)) { lookahead = parser.value(optimizationLookahead).toInt(); }
    else { lookahead = 1.0; }

    if (checkParserValue(parser, optimizationSpeedPriorityFactor, "", 0.0)) {optimize_speedfactor = parser.value(optimizationSpeedPriorityFactor).toDouble(); }
    else { optimize_speedfactor = 0.0;}

    try {
        std::cout << "Reading Trains!                 \r";

        SimulatorAPI::ContinuousMode::createNewSimulationEnvironmentFromFiles(
            QString::fromStdString(nodesFile),
            QString::fromStdString(linksFile),
            NETWORK_NAME, QString::fromStdString(trainsFile),
            timeStep);

        auto trains = SimulatorAPI::ContinuousMode::getTrains(NETWORK_NAME);

        for (auto &t: trains) {
            QEventLoop::connect(t.get(), &Train::slowSpeedOrStopped,
                    [](const auto &msg){
                        ErrorHandler::showWarning(msg);});

            QEventLoop::connect(t.get(), &Train::suddenAccelerationOccurred,
                    [](const auto &msg){
                ErrorHandler::showWarning(msg);});

            t->setOptimization(optimize, optimize_speedfactor,
                               optimizerFrequency, lookahead);
        }
        Simulator* sim =
            SimulatorAPI::ContinuousMode::getSimulator(NETWORK_NAME);

        if (exportLocation != "" ) { sim->setOutputFolderLocation(exportLocation); }
        if (summaryFilename != "" ) { sim->setSummaryFilename(summaryFilename); }

        sim->setExportInstantaneousTrajectory(exportInstaTraj,
                                              instaTrajFilename);

        if (parser.isSet(interactiveOption)) {
            runInteractive(sim);
            std::cout << "Output folder: " << sim->getOutputFolder() << std::endl;
            return 0;
        }

        // run the actual simulation
        std::cout <<"Starting the Simulator!                                "
                     "              \n";
        sim->runSimulation();
        std::cout << "Output folder: " << sim->getOutputFolder() << std::endl;

        // qDebug() << "\nType name for 65537:" << QMetaType::typeName(65537);
    } catch (const std::exception& e) {
        ErrorHandler::showError(e.what());
        return 1;
    }
    return 0;
}
