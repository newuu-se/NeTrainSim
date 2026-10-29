//
// Created by Ahmed Aredah
// Version 0.0.1
//

#include <cmath>
#include <iostream>
#include "car.h"
#include "traintypes.h"
#include "energyconsumption.h"
#define stringify( name ) #name

using namespace std;

Car::Car(double carLength_m, double carDragCoef,
         double carFrontalArea_sqm, double carEmptyWeight_t,
         double carCurrentWeight_t, int carNoOfAxiles, int carType,
         double auxiliaryPower_kw,
         double batteryMaxCapacity_kwh,
         double batteryInitialCharge_perc,
         double tenderMaxCapacity_l,
         double tenderInitialCapacity_perc,
         std::string carName) {
    // assign the values to the variables
	this->name = carName;
    this->length = carLength_m; // provided length is in meters
    this->dragCoef = carDragCoef; // unitless
    this->frontalArea = carFrontalArea_sqm; // square meters
    this->emptyWeight = carEmptyWeight_t;   // light weight of the car in tons. Define before currentWeight
    this->currentWeight = carCurrentWeight_t; // gross weight of the car in tons
    this->noOfAxiles = carNoOfAxiles;         // unitless (count)
    this->carType = static_cast<TrainTypes::CarType>(carType); // convert from digits to enum type
    this->auxiliaryPower = auxiliaryPower_kw;                   // power in kw

    // if the car is assigned to cargo, set the battery and the tank to zeros
    if (this->carType == TrainTypes::CarType::cargo){
        this->setBattery(0.0, 0.0, 1.0, EC::DefaultCarBatteryCRate);
        this->SetTank(0.0, 0.0, EC::DefaultCarMinTankDOD);
    } //end if
    // if it is in rechargable technologies, only assign a battery capacity and discard the tank
    else if (TrainTypes::carRechargableTechnologies.exist(this->carType) &&
             this->carType != TrainTypes::CarType::cargo) {
        this->setBattery(batteryMaxCapacity_kwh, batteryInitialCharge_perc,
                         EC::DefaultCarBatteryDOD, EC::DefaultCarBatteryCRate);
        this->SetTank(0.0, 0.0, EC::DefaultCarMinTankDOD);
    } // end else if
    // if it is non-recharable technology, add tank capacity only and discard the battery
    else if (TrainTypes::carNonRechargableTechnologies.exist(this->carType) &&
             this->carType != TrainTypes::CarType::cargo) {
        this->setBattery(0.0, 0.0, 1.0, EC::DefaultCarBatteryCRate);
        if (this->carType == TrainTypes::CarType::hydrogenFuelCell &&
            std::isnan(tenderMaxCapacity_l))
        {
            tenderMaxCapacity_l = EC::DefaultCarTenderMaxCapacityForHydrogen;
        }
        else if (this->carType != TrainTypes::CarType::hydrogenFuelCell &&
                   std::isnan(tenderMaxCapacity_l))
        {
            tenderMaxCapacity_l = EC::DefaultCarTenderMaxCapacity;
        }
        this->SetTank(tenderMaxCapacity_l, tenderInitialCapacity_perc, EC::DefaultCarMinTankDOD);
        double fuelWeight = 0.0; // in ton
        // if the tender is for diesel
        if (this->carType == TrainTypes::CarType::dieselTender) {
            fuelWeight = this->getTankInitialCapacity() * EC::DefaultDieselDensity;
        }
        // if the tender is for biodiesel
        else if (this->carType == TrainTypes::CarType::biodieselTender) {
            fuelWeight = this->getTankInitialCapacity() * EC::DefaultBioDieselDensity;
        }
        else if (this->carType == TrainTypes::CarType::hydrogenFuelCell) {
            fuelWeight = this->getTankInitialCapacity() * EC::DefaultHydrogenDensity;
        }
        // update the weight to account for the fuel weight
        this->currentWeight = this->emptyWeight + fuelWeight;

    } // end else if

    if (this->carType == TrainTypes::CarType::cargo) {
        this->brakedWeightRatio = EC::DefaultCarBrakedWeightRatio_Cargo;
    } else {
        this->brakedWeightRatio = EC::DefaultCarBrakedWeightRatio_Tender;
    }

    this->hostLink = std::shared_ptr<NetLink>(); // assign empty placeholder
    this->trackCurvature = 0; // zero initialized
    this->trackGrade = 0;   // zero initialized
};

void Car::setCarCurrentWeight(double newCurrentWeight) {
    // todo: raise error if the empty weight is > gross weight
    if (newCurrentWeight > this->emptyWeight) {
        this->currentWeight = newCurrentWeight;
    } // end if
}

double Car::getCargoNetWeight() {
    // the cargo Net Weight is the weight of the carried commodities only.

    // check if the car of cargo type, return its net weight if it is
	if (this->carType == TrainTypes::CarType::cargo) {
		return this->currentWeight - this->emptyWeight;
    } // end if
    // return zero if not
	return 0.0;
}

double Car::getResistance(double trainSpeed) {
	// these calculations depend of US units, so these are the conversions factors from meteric system
    // They are listed this way and not converted outside for a better tracing/changing


    // Convert train speed from m/s to km/h for the Grebenyuk formula (u)
    double u = trainSpeed * 3.6;

    // Convert locomotive weight from metric tonnes to kg (m)
    double m = this->currentWeight * 1000.0;

    // Acceleration due to gravity (m/s^2)
    const double g = 9.80665;

    // Weight force in Newtons (mg)
    double mg = m * g;

    // 1. Grebenyuk (1987) rolling and aerodynamic resistance
    double grebenyukRes = (1.1 + 0.01 * u + 0.000227 * (u * u)) * mg * 1e-3;

    // 2. Grade resistance
    double gradeRes = mg * (this->trackGrade / 100.0);

    // 3. Curve resistance, 1520 mm gauge traction rules: 700 / R (N/kN).
    // trackCurvature is in percent (100 / R, R = curve radius in m), so
    // 700 / R = 7 * curvature. Zero on straight track; left/right sign ignored.
    double curveRes = 7.0 * std::abs(this->trackCurvature) * mg * 1e-3;

    // Total resistance in Newtons
    double rVal = grebenyukRes + gradeRes + curveRes;

    return rVal;
}

double Car::getEnergyConsumption(double &timeStep) {
    // return the auxiliary energy consumption by the time step.
    return this->auxiliaryPower* timeStep* (3600 / 1000); // time step is in seconds
}


std::pair<bool, double> Car::consumeFuel(double timeStep, double trainSpeed,
                                         double EC_kwh,
                                         double carVirtualTractivePower,
                                         double dieselConversionFactor,
                                         double biodieselConversionFactor,
                                         double hydrogenConversionFactor, double dieselDensity,
                                         double biodieselDensity, double hydrogenDensity) {
    // clear the variables for reassignning
    this->energyConsumed = 0.0; // the step energy consumption of 1 tech
    this->energyRegenerated = 0.0; // the step energy regeneration of 1 tech
    this->carbonDioxideEmission = 0.0;

    // check if the energy consumption (in kWH) is energy consumption not regenerated energy
    if (EC_kwh > 0.0) {
        // consume diesel if diesel tender
        if (this->carType == TrainTypes::CarType::dieselTender) {
            return this->consumeFuelDiesel(EC_kwh, dieselConversionFactor, dieselDensity);
        } // end if technology type diesel
        // consume bio diesel if bio diesel tender
        else if (this->carType == TrainTypes::CarType::biodieselTender) {
            return this->consumeFuelBioDiesel(EC_kwh, biodieselConversionFactor, biodieselDensity);
        }// end if technology type biodiesel
        else if (this->carType == TrainTypes::CarType::batteryTender) {
            return this->consumeElectricity(timeStep, EC_kwh);
        }// end if technology type battery
        else if (this->carType == TrainTypes::CarType::hydrogenFuelCell) {
            return this->consumeFuelHydrogen(EC_kwh, hydrogenConversionFactor, hydrogenDensity);
        }// end if technology type hydrogen
        return std::pair(false, EC_kwh);
    }// end if EC_kwh
    return std::pair(false, EC_kwh);  // cannot consume and return full requested energy
}


double Car::getMaxProvidedEnergy(double &timeStep) {
    if (TrainTypes::carRechargableTechnologies.exist(this->carType)) {
        if (this->hostLink->hasCatenary) {
            return std::numeric_limits<double>::infinity();
        }
        return this->getBatteryMaxDischarge(timeStep);
    }
    else if (TrainTypes::carNonRechargableTechnologies.exist(this->carType)) {
        if (! this->tankHasFuel()) {
            return 0.0;
        }
    }
    return std::numeric_limits<double>::infinity();
}

bool Car::canProvideEnergy(double &EC, double &timeStep) {
    if (EC < 0.0) {return true; }
    if (TrainTypes::carRechargableTechnologies.exist(this->carType)) {
        if (EC <= this->getBatteryMaxDischarge(timeStep) && this->isBatteryDrainable(EC)) {
            return true; // the car type cannot provide energy
        }
    }
    else if (TrainTypes::carNonRechargableTechnologies.exist(this->carType)) {
        if (this->isTankDrainable(EC)) {
            return true;
        }
    }
    return false;
}


ostream& operator<<(ostream& ostr, Car& car) {
	ostr << "Rail Car:: length: " << car.length << ", drag: " << car.dragCoef << ", frontal area: " << car.frontalArea;
	ostr << ", car type: " << TrainTypes::carTypeToStr(car.carType) << ", current weight: " << car.currentWeight;
	ostr << ", total weight: " << car.emptyWeight << ", no axles: " << car.noOfAxiles << endl;
	return ostr;
};
