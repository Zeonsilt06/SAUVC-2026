#include "depth.h"

#define MBAR_PER_CM 0.980665

DepthController::DepthController() :
    pressureSensor(ADDRESS_HIGH),
    surfacePressure(0),
    currentDepth(0)
{
}

bool DepthController::begin(TwoWire &wire)
{
    wire.begin();
    wire.setClock(50000);

    pressureSensor.reset();
    delay(20);

    if (pressureSensor.begin(wire) != 0)
        return false;

    pressureSensor.getTemperature(CELSIUS, ADC_512);
    delay(10);
    surfacePressure = pressureSensor.getPressure(ADC_2048);

    return true;
}

void DepthController::update()
{
    pressureSensor.getTemperature(CELSIUS, ADC_512);
    float pressure = pressureSensor.getPressure(ADC_2048);
    computeDepth(pressure);
}

void DepthController::computeDepth(float pressure)
{
    static float filteredPressure = 0;
    if (filteredPressure == 0)
        filteredPressure = pressure;

    filteredPressure = 0.9 * filteredPressure + 0.1 * pressure;

    currentDepth = (filteredPressure - surfacePressure) / MBAR_PER_CM;
}

float DepthController::getDepth()
{
    return currentDepth;
}
