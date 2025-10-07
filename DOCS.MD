# Program for predicting ideal shifting times on a Raspberry Pi 3

This script receives a CAN message and reads the RPM value it holds. Using the
value it predicts when the driver needs to shift gears.
It calculates the future RPM (which we will call predictedRPM for simplicity)
using the formula for 2nd order Taylor expansion:

$$
n(t + \Delta{t}) = n(t) + n'(t)\Delta{t} + \frac{n''(t)}{2}\Delta{t}^2
$$

Where:

- ( n(t) ) RPM at time ( t )
- ( n'(t) ) First derivative of RPM at time ( t ) or rate of change of RPM
- (0.5 \* n''(t) ) Second derivative of RPM at time ( t ) or acceleration of RPM

> [!NOTE]
> !!! Some of these values could change in the future, for now these ones are used for testing !!!

```
currentRPM

UpShiftThreshold = 13500;
DownShiftThreshold = 2000;

sampleTime = 0.05
predictionTime = 0.3;
alphaRPM = 0.2
alphaDotRPM = 0.2
alphaDDotRPM = 0.2
rpmFiltered = 0
rpmDotFiltered = 0
rpmDDotFiltered = 0

lastRpm
lastRpmDot
lastTick
```

## Pseudocode

```
Main()
{
    while(true){
        if((getTick() - lastTick) >= sampleTime){
            currentRPM = GetCurrentRpm();
            FilterValues();
            predictedRPM = predictRPM(predictionTime);
            if(predictedRPM >= UpShiftThreshold && rpmDot > 0){
                PlayUpshiftSound();
            }
            if(predictedRPM <= DownShiftThreshold && rpmDot < 0){
                PlayDownShiftSound();
            }
        }
    }
}
```

### Explenations of variables and functions:

- rpmDot is the first derivative of the RPM value and in the if statements
  we use them to determine if the RPM is rising and falling so we don’t get
  false upshifts and downshifts
- FilterValues will calculate the derivates for the formula
- PredictRPM will calculate the rpm

### Function for predicting the RPM

```
var PredictRPM(predictionTime)
{
return (rpmFiltered + rpmDotFiltered * predictionTime + 0.5 * rpmDDotFiltered * predictionTime^2);
}
```

> [!NOTE]
> We first use EMA(Exponential moving average) to filter the values of RPM and
> calculate the derivatives for the prediction formula

### Function for filtering the values

```
void FilterValues():
//if the program starts when the rpm is already bigger than 0 it sets to current RPM
    if first_run == 0 and rpm > 0:
        rpmFiltered = currentRpm
        lastRpm = currentRpm
        first_run = 1
// 1. Exponential smoothing of RPM
    rpmFiltered = alphaRpm * currentRpm + (1 - alphaRpm) * rpmFiltered
// 2. First derivative
    rpmDot = (rpmFiltered - lastRpm) / sampleTime
    rpmDotFiltered = alphaDot * rpmDot + (1 - alphaDot) * rpmDotFiltered
// 3. Second derivative
    rpmDDot = (rpmDotFiltered - lastRpmDot) / sampleTime
    rpmDDotFiltered = alphaDDot * rpmDDot + (1 - alphaDDot) * rpmDDotFiltered
// 4. Update last values
    lastRpm = rpmFiltered
    lastRpmDot = rpmDotFiltered
```

> [!WARNING]
> This is just a pseudocode, it is not meant to be run as is. It is just to
> explain the logic behind the program. The actual implementation is done in python and cpp

## Python requirements

> [!NOTE]
> The program needs sox python package to work with CAN bus

Command to install it:

```bash
pip install sox
```
